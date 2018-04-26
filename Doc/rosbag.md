### 1 record

#### 1.1 -a, --all
Record all topics.
```
rosbag record -a
```

#### 1.2 -d, --duration
Specify the maximum duration of the recorded bag file.
```
$ rosbag record --duration=30 /chatter
$ rosbag record --duration=5m /chatter
$ rosbag record --duration=2h /chatter
```

#### 1.3 -o PREFIX, --output-prefix=PREFIX
Prepend PREFIX to beginning of bag name before date stamp.
```
rosbag record -o session1 /chatter
```

#### 1.4 -O NAME, --output-name=NAME
Record to bag with name NAME.bag.
```
rosbag record -O session2_090210.bag /chatter
```

#### 1.5 --split
Split the bag when maximum size or duration is reached
```
rosbag record --split --size=1024 /chatter
rosbag record --split --duration=30 /chatter
rosbag record --split --duration=5m /chatter
rosbag record --split --duration=2h /chatter
```

### 2 info <bag-files>

Display a summary of the contents of the bag files.
```
rosbag info session*.bag
```

### 3 play

#### 3.1 -l, --loop
Loop playback.
```
rosbag play -l recorded1.bag
```
