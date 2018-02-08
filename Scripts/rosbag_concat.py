#!/usr/bin/env python2

import rosbag

import sys
import os
from optparse import OptionParser

parser = OptionParser()
parser.add_option('-f', '--file', dest='list_file',
                  help='File containing the list of bag files to concatenate',
                  metavar='FILE')
parser.add_option('-o', '--output', dest='output_file', default='output.bag',
                  help='Name of the output file',
                  metavar='FILE')

(options, args) = parser.parse_args()

files = []
if options.list_file != None:
    with open(options.list_file) as f:
        files = map(lambda l: l.rstrip(), f.readlines())
elif len(args) > 0:
    files = args
else:
    sys.stderr.write('Not enough arguments!\n')
    sys.exit(1)

with rosbag.Bag(options.output_file, 'w') as outbag:
    for bag_file in files:
        print('Now processing ' + os.path.basename(bag_file))
        for topic, msg, t in rosbag.Bag(bag_file).read_messages():
            outbag.write(topic, msg, t)
