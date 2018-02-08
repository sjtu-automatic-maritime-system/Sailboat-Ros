import pandas as pd
import rosbag_pandas
dataframe = rosbag_pandas.bag_to_dataframe('/home/sjtu/BAG/gps_test_0424_2017-04-24-20-52-44.bag') #awesome data processing
dataframe.to_csv('gps_test.csv')
print(dataframe)
print(111)
