import argparse, csv, rosbag2_py
from datetime import datetime
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader


def get_field_value(msg, field_name):
    try:
        fields = field_name.split('.')
        value = msg
        for field in fields:
            value = getattr(value, field)
        return value
    except AttributeError:
        raise ValueError(f"Field '{field_name}' not found in message type {type(msg).__name__}")


def main():
    parser = argparse.ArgumentParser(description="Read data from rosbag2")
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "--topic", 
        default="/temperature", 
        help="topic name to read from (default: /temperature)"
    )
    parser.add_argument(
        "--field",
        default="temperature",
        help="message field to plot (supports dot notation for nested fields, e.g., 'linear.x')"
    )
    parser.add_argument(
        "--output",
        default="data",
        help="save file name (optional)"
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="print min, max, avg of the message values (optional)"
    )

    args = parser.parse_args()
    timestamps = []
    values = []
    
    for topic, msg, timestamp in read_messages(args.input):
        if topic == args.topic:
            try:
                time_seconds = timestamp / 1e9
                field_value = get_field_value(msg, args.field)
                timestamps.append(time_seconds)
                values.append(field_value)
            except ValueError as e:
                print(f"Error extracting field '{args.field}': {e}")
                return
    
    if not timestamps:
        print(f"No data found for topic {args.topic}")
        return
    
    start_time = timestamps[0]
    relative_times = [(t - start_time) for t in timestamps]
    
    avg_value = sum(values) / len(values)
    min_value = min(values)
    max_value = max(values)
    
    with open(f"{args.output}.csv", 'w', newline='') as f:
        msg = args.field
        
        if args.stats:
            fieldnames = ['time', msg, f'average_{args.field}', 'minimum', 'maximum']
        else:
            fieldnames = ['time', msg]
            
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        
        if args.stats:
            writer.writerow({
                'time': 'n/a',
                msg: 'n/a',
                f'average_{args.field}': avg_value,
                'minimum': min_value,
                'maximum': max_value
            })
            writer.writerows(
                {'time': rt, msg: v} for rt, v in zip(relative_times, values)
            )
            print(f"Data with statistics written to {args.output}.csv")
        else:
            writer.writerows(
                {'time': rt, msg: v} for rt, v in zip(relative_times, values)
            )   
            print(f"Data written to {args.output}.csv")
    
if __name__ == "__main__":
    main()