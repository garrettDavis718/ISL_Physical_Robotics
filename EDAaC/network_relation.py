import os
import numpy as np
import pandas as pd
import math
from matplotlib import pyplot as plt
import time

from utils import *

def convert(list1):   ## converting nasty str to a usable int format
        if list1 == None:
                return list1
        listFinal = []
        listTemp = []
        neg = [list1[1],list1[13],list1[25]]
        listTemp.extend([list1[2:12],list1[14:24],list1[26:36]])
        for x in range(3):
                if neg[x] == '-':
                        listFinal.append(-float(listTemp[x]))
                else:
                        listFinal.append(float(listTemp[x]))
        return listFinal



def network_relation(root, files): ## Powerhouse method which manipulates data for relation

        ## Only use this when files is empty and you want to select all CSV files in the given directory (root)
        #files = [f for f in os.listdir(root) if f.endswith('.csv')]
        start = time.time()
        
        ## Get the max number of people accross all the files
        num_people = max([len(get_data(root, f)['Object']) for f in files])
        
        ## Initialize and empty correspondence Pandas DataFrame
        correspondence = pd.DataFrame(np.random.randint(0,5,size=(num_people,
        len(files))), columns = [file.split('.csv')[0] for file in files],
        index=['Person_'+str(x) for x in range(num_people)])
        
        correspondence[:] = np.nan
        
        ## Checks to see if all files are csv files
        for file in files:
                if not file.endswith('.csv'):
                        print("Files must be csv files")
                        quit()
        
        ## Each iteration, we compare the first and second files in the files list, 
        ## then remove the first one so that the next two are compared. Therefore
        ## we keep repeating this process until there is only one file left, and
        ## since we cannot compare it to a second one, it is the last and we stop
        first = True
        while len(files) > 1:
                first_file = files[0]
                second_file = files[1]

                ## Obtain data and place in Pandas DataFrame (get_data is in untils.py)
                df_1 = get_data(root, first_file)
                df_2 = get_data(root, second_file)

                ## If the two shapes are different, add None values to make them the same
                if df_1.shape[0] != df_2.shape[0]:
                        print('Error')
                        if df_1.shape[0] > df_2.shape[0]:
                                df_2.loc[len(df_2.index)] = [None, None, None] 
                        else:
                                df_1.loc[len(df_1.index)] = [None, None, None]
                                
                temp = pd.DataFrame(np.random.random_sample(size=(num_people, num_people)), columns = df_1['Object'].to_list(), index=df_2['Object'].to_list())

                df_1_nodelist = []
                df_2_nodelist = []


                for i in range(len(df_1)):
                        df_1_nodelist.append([df_1['Class Confidence'][i], df_1['3D_Bounding_Box'][i]])

                for j in range(len(df_2)):
                        df_2_nodelist.append([df_2['Class Confidence'][j], df_2['3D_Bounding_Box'][j]])

                for key1, node1 in enumerate(df_1_nodelist):
                        for key2, node2 in enumerate(df_2_nodelist):
                                loss = get_loss_2box(node1[1], node1[0], node2[1], node2[0])
                                temp.at[df_2['Object'][key2], df_1['Object'][key1]] = loss
                        
                cols, rows, _ = hungarian(temp.to_numpy())
                if first:
                        correspondence[first_file.split('.csv')[0]] = cols
                        correspondence[second_file.split('.csv')[0]] = rows
                else:
                        correspondence = update_correspondence(cols, rows, correspondence, first_file.split('.csv')[0], second_file.split('.csv')[0])


                files=files[1:]
                first = False
        end = time.time()
        print(f"Total time: {end-start}")
        correspondence = correspondence.astype(int)
        print(correspondence)
        correspondence.to_csv('output_relation.csv')
        return [temp.to_numpy()[cols, rows].sum(), cols]


## input filepaths here, the relation will be displayed in the terminal
files = ['data_exp_testFiles-1pos_3-2.83--5+rot_0-1.57-0.csv', 'data_exp_testFiles-1pos_2-2.83--9+rot_0-2.36-0.csv']
root = 'C:/Users/Eppat/Documents/College/UNCFSU/ISL Research/TurtleBotZED'

print(network_relation(root, files))
