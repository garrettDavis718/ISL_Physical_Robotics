import os
import numpy as np
import pandas as pd
import nums_from_string
from pytorch3d.ops import box3d_overlap
import torch
from scipy.spatial import ConvexHull
import math

from hungarian import linear_sum_assignment
from scipy.spatial.distance import directed_hausdorff


class Node:
	"""Node class holds information regarding a node with edges on a graph

	An instance of this class represents a node on a graph, which has edges, represented
	as vectors (numpy lists). It has the ability to calculate the angle between two vectors, 
	using the calculateAngle method. Each instance also holds its position
	"""
	def __init__(self, position, bbox3d, class_conf):
		"""Constructor class which is used to create a Node instance
		
		Accepts a mandatory position param and two optional params, vectors and angles.
		
		Args:
			position (1x3 np array): This holds the Cartesian coordinate information for location of this node
			vectors (list of 1x3 np arrays): This holds all vectors (edges) which go from this node to another
			angles (list of ints): This holds all angles between vectors 
		
		"""
		self.position = position  ## np format
		self.bbox3d = bbox3d
		self.class_conf = class_conf
	def getPosition(self):
		return self.position
	def getDistance(self, node1):
		if (node1.getPosition() == [None, None, None]).all():
			return 100
		elif (self.getPosition() == [None, None, None]).all():
			return 100
		else:
			return np.linalg.norm(self.getPosition() - node1.getPosition())

def preprocessing(string):
	lst = [nums_from_string.get_nums(str) for str in string.splitlines()]
	return np.array(lst)


def get_data(root, file_name):
	data_frame = pd.read_csv(os.path.join(root, file_name))
	data_frame['Object'] = file_name.split('.csv')[0] + '_' + data_frame['Unnamed: 0'].astype(str)
	df = data_frame[['Object','3D_Bounding_Box','Object_Position', 'Class Confidence']].copy()
	
	if type(df['3D_Bounding_Box'][0]) == str:
		df['3D_Bounding_Box'] = df['3D_Bounding_Box'].map(preprocessing)

	return df


def hausdorff(u, v):
	return directed_hausdorff(u,v)


def hungarian(matrix):
	row_ind, col_ind = linear_sum_assignment(matrix)
	return col_ind, row_ind, matrix[row_ind, col_ind].sum()


def update_correspondence(cols, rows, correspondence, first_file, second_file):
	
	if not correspondence[first_file].isnull().all():
		for i in cols:
			lst_idx = np.where(cols==i)
			mat_idx = correspondence.loc[correspondence[first_file] == i].index
			correspondence.at[mat_idx, second_file] = rows[lst_idx]
	
	return correspondence


def get_loss_2box(bbox1, classconf1, bbox2, classconf2):
	## Takes two arguments, bbox1 and bbox2, where they both represent two cubiods in 3d space
	## Below is a section of the documentation for the box3d_overlap function from pytorch3d.ops.
	## The constraints given below apply to the argument boxes. 
	""" 
	Computes the intersection of 3D boxes1 and boxes2.

	Inputs boxes1, boxes2 are tensors of shape (B, 8, 3)
	(where B doesn't have to be the same for boxes1 and boxes2),
	containing the 8 corners of the boxes, as follows:

	(4) +---------+. (5)
	    | ` .     |  ` .
	    | (0) +---+-----+ (1)
	    |     |   |     |
	(7) +-----+---+. (6)|
	    ` .   |     ` . |
	    (3) ` +---------+ (2)


	NOTE: Throughout this implementation, we assume that boxes
	are defined by their 8 corners exactly in the order specified in the
	diagram above for the function to give correct results. In addition
	the vertices on each plane must be coplanar.
	"""
	## In other words, each individual bounding box MUST be a perfect cuboid, with all 90 degree corners. 
	## However, the two bounding boxes do not have the be on the same planes, so the angles between eachother's
	## respective correlating planes can differ.

	## This function returns a single loss value, defined by GIoU box and class loss
	
	## Example Bounding box 1
	"""
	bbox1 = [
	    [1.0, 1.0, 1.0], [2.0, 1.0, 1.0],
	    [2.0, 1.0, 2.0], [1.0, 1.0, 2.0],
	    [1.0, 0.0, 1.0], [2.0, 0.0, 1.0],
	    [2.0, 0.0, 2.0], [1.0, 0.0, 2.0]
	]
	"""

	## Example Bounding box 2
	"""
	bbox2 = [
	    [1.0, 1.0, 1.0], [2.0, 1.0, 2.0],
	    [1.0, 1.0, 3.0], [0.0, 1.0, 2.0],
	    [1.0, 0.0, 1.0], [2.0, 0.0, 2.0],
	    [1.0, 0.0, 3.0], [0.0, 0.0, 2.0]
	]
	"""
	## The format needs to have an additional dimension (required by box3d_overlap, which is then converted to numpy and then to tensor
	## The initial numpy conversion conserves resources and time for the converstion to tensor, then all values inside are converted to floats,
	## where the get_data function automatically converts them to doubles due to the length of some.
	bbox1 = torch.tensor(np.array([bbox1])).float()
	bbox2 = torch.tensor(np.array([bbox2])).float()

	## If the class confidences are not in the 0 to 1 range, divide them by 100 
	if classconf1 > 1:
	      classconf1 = classconf1/100

	if classconf2 > 1:
	      classconf2 = classconf2/100

	## As stated before, the tensors need to be 3rd dimensional, final check before continuing
	if len(bbox1.shape) != 3 | len(bbox1.shape) != 3:
	    raise Exception("Please provide correct list dimensions! Must be a 2 dimensional list") ## 2-d because we add a dimension in the previous lines

	## Calculate the volume of the intersection as well as IoU
	intersection = box3d_overlap(bbox1, bbox2)
	intersection_volume = intersection[0].item()
	IoU = intersection[1].item()

	## First calculate the volume of each bounding box, then 
	## Combine all the points which define the boxes, returns smallest shape which holds all points
	bbox1_volume = ConvexHull((bbox1[0]).detach().cpu().numpy())
	bbox2_volume = ConvexHull((bbox2[0]).detach().cpu().numpy())
	hull = ConvexHull(torch.cat((bbox1[0], bbox2[0])).detach().cpu().numpy())

	## Calculate Union and GIoU
	union = bbox1_volume.volume + bbox2_volume.volume - intersection_volume
	GIoU = IoU - ((hull.volume - union) / hull.volume)

	## Calculate respective losses
	GIoU_loss = 1 - GIoU
	class_loss = -math.log(classconf1) + -math.log(classconf2)

	## Since they are on similar scale, with values in similar ranges, it is safe to summate them. 
	total_loss = GIoU_loss + class_loss
	return total_loss
