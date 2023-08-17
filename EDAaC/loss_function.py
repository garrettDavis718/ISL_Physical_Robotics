from pytorch3d.ops import box3d_overlap
import torch
from scipy.spatial import ConvexHull
import math

def get_loss_2box(bbox1, classconf1, bbox2, classconf2):
      ## Takes two arguments, bbox1 and bbox2, where they both represent two cubiods in 3d space
      ## Below is a section of the documentation for the box3d_overlap function from pytorch3d.ops.
      ## The constraints given below apply to the argument boxes.
      """
      Computes the intersection of 3D boxes1 and boxes2.

      Inputs boxes1, boxes2 are tensors of shape (B, 8, 3)
      (where B doesn't have to be the same for boxes1 and boxes2),
      containing the 8 corners of the boxes, as follows:

      (4) +----- ----+. (5)
            | ` .         |  ` .
            | (0) +---+----- + (1)
            |       |    |         |
      (7) +-----+---+. (6) |
              ` .   |         ` .   |
          (3)   ` +---- -- -- -+ (2)


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
          raise Exception("Please provide correct list dimensions! Must be a 2 dimensional list") ## 2-d argument because we add a dimension in the previous lines (we add a dimension)

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
      print(f"\nGIoU Loss: {GIoU_loss}")
      print(f"Class Loss: {class_loss}")
      return total_loss