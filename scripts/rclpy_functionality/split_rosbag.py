import rosbag2_py


# Path to the bag file
bag_file_path = '/home/siyuchen/catkin_ws/' \
                'imu_raw_data_bag_recorder/imu_raw_data_bag_recorder_0.mcap'

# Create a reader
reader = rosbag2_py.SequentialReader()

# Create a writer
writer = rosbag2_py.SequentialWriter()

# Open the bag file
storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='mcap')
reader.open(storage_options, rosbag2_py.ConverterOptions())

# bag_name = 'bag_split_1'
# start_ts = 1688379045.1
# end_ts = 1688379215.1

# bag_name = 'bag_split_2'
# start_ts = 1688379397.1
# end_ts = 1688379737.1

bag_name = 'bag_split_3'
start_ts = 1688379747.1
end_ts = 1688379934.1

storage_options = rosbag2_py._storage.StorageOptions(
    uri=bag_name,
    storage_id='mcap')
converter_options = rosbag2_py._storage.ConverterOptions('', '')
writer.open(storage_options, converter_options)

# Iterate through messages
for topic_metadata in reader.get_all_topics_and_types():
    print(f'topic: {topic_metadata.name}, type: {topic_metadata.type}')
    writer.create_topic(topic_metadata)

# Read messages
while reader.has_next():
    (topic, msg_bytes, t) = reader.read_next()
    ts = t / 1e9
    if start_ts < ts < end_ts:
        writer.write(topic, msg_bytes, t)
