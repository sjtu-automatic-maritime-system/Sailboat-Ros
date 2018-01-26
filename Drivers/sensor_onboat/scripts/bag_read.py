import rosbag
import csv
bag = rosbag.Bag('/home/sjtu/BAG/gps_test_0424_2017-04-24-20-52-44.bag')
# print(bag)
f = open('gps_test.csv', 'wb')
writer = csv.writer(f)
for topic, msg, t in bag.read_messages(topics=['/WTST_tmp', '/gps_kf']):
    # print('in loop')
    print(type(msg))
    writer.writerow(msg)

print('bag close...')
f.close()
bag.close()


# https://github.com/unl-nimbus-lab/bag2csv/blob/master/bag_reader.py