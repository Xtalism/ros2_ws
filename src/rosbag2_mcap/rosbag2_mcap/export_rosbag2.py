import argparse, rosbag2_py, csv, matplotlib.pyplot as plt
from datetime import datetime
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def parse_args():
    parser = argparse.ArgumentParser(description="Graph data from rosbag2")
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "--topic", 
        help="topic name to read from"
    )
    parser.add_argument(
        "--field",
        help="message field to plot (supports dot notation for nested fields, e.g., 'linear.x')"
    )
    parser.add_argument(
        "--output",
        help="save plot name file (optional)"
    )
    parser.add_argument(
        "--filetype",
        help="plot filetype (optional)"
    )
    parser.add_argument(
        "--stats",
        help="include statistics (average, min, max) in output (optional)",
    )
    parser.add_argument(
        "--graph",
        action="store_true",
        help="generate a graph (optional)",
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help="export data to CSV (optional)",
    )

    args = parser.parse_args()
    return args

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

def stats_rosbag2(args):
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
    return avg_value, min_value, max_value, values, relative_times

def graph_rosbag2():
    args = parse_args()
    stats = stats_rosbag2(args)

    if stats is None:
        return

    avg_value, min_value, max_value, values, relative_times = stats

    plt.figure(figsize=(12, 6))
    plt.plot(relative_times, values, 'b-', linewidth=1.5, marker='o', markersize=3)
    plt.xlabel('Time (seconds)')
    plt.ylabel(args.field.title())
    plt.title(f'{args.field.title()} data from {args.topic}')
    plt.grid(True, alpha=0.3)

    if args.stats:
        stats_rosbag2(values)
    
    plt.text(0.02, 0.98, 
             f'Avg: {avg_value:.2f}\nMin: {min_value:.2f}\nMax: {max_value:.2f}', 
             transform=plt.gca().transAxes, 
             verticalalignment='top',
             bbox=dict(boxstyle='square', facecolor='white', alpha=0.8))
    
    plt.tight_layout()
    
    if args.output:
        if args.filetype:
            filename = f"{args.output}.{args.filetype}"
            plt.savefig(filename, dpi=300, bbox_inches='tight', format=args.filetype)
            print(f"Plot saved to {filename}")
        else:
            filename = args.output
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Plot saved to {filename}")
    elif args.filetype and not args.output:
        raise ValueError("Error: --filetype specified without --output. Please provide an output filename.")
    else:
        plt.show()
    
    print(f"Plotted {len(values)} {args.field} readings")
    print(f"Duration: {relative_times[-1]:.2f} seconds")
    
def csv_rosbag2():
    args = parse_args()
    stats = stats_rosbag2(args)
    
    if stats is None:
        return
        
    avg_value, min_value, max_value, values, relative_times = stats
    
    if not args.output:
        raise ValueError("Error: --output FILENAME is required for CSV export.")
        return
    
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


def main():
    args = parse_args()
    if args.graph:
        graph_rosbag2()
    elif args.csv:
        csv_rosbag2()
    else:
        raise ValueError("Error: Please specify either --graph or --csv option.")


if __name__ == "__main__":
    main()