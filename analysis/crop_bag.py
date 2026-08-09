#!/usr/bin/env python3
"""
Crop the first N seconds of a rosbag2 (sqlite3) file, handling decompression correctly
(compression_mode: MESSAGE) to remain compatible with evo.

Usage:
    python3 crop_bag.py <input_bag_dir> <output_bag_dir> <seconds_to_cut>

Example:
    python3 crop_bag.py UKF16 UKF16_cropped 22
"""
import sys
import rosbag2_py
import zstandard

ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"
_zstd_decompressor = zstandard.ZstdDecompressor()


def maybe_decompress(data: bytes) -> bytes:
    if bytes(data[:4]) == ZSTD_MAGIC:
        return _zstd_decompressor.decompress(bytes(data))
    return data


def crop_bag(input_uri: str, output_uri: str, cut_seconds: float):
    storage_options_in = rosbag2_py.StorageOptions(uri=input_uri , storage_id="sqlite3 ") 
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr "
    )
 
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options_in, converter_options)

    topics_and_types = reader.get_all_topics_and_types() 

    # Determine the actual start timestamp of the bag
    metadata = reader.get_metadata()
    start_ns = metadata.starting_time.nanoseconds 
    cutoff_ns = start_ns +  int(cut_seconds * 1e9)

    # Writer WITHOUT compression -> avoids any decompression issues on the evo side 
    storage_options_out =  rosbag2_py.StorageOptions(uri=output_uri, storage_id="sqlite3")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options_out, converter_options)
 
    for topic_metadata in topics_and_types:
        writer.create_topic(topic_metadata)

    kept,skipped=0, 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t <  cutoff_ns:
            skipped += 1 
            continue  
        data  = maybe_decompress(data)
        writer.write(topic, data, t)
        kept += 1

    print(f"Done.Messages kept  : {kept}, skipped: {skipped}")
    print(f"New bag :  {output_uri}")


if __name__ == "__main__": 
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1) 
    crop_bag(sys.argv[1], sys.argv[2], float(sys.argv[3]))
