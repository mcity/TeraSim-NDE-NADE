import argparse
import boto3
import glob
import os


parser = argparse.ArgumentParser(description='Run simulation.')
parser.add_argument('--dir', type=str, help='output directory', default="output")
parser.add_argument('--mode', type=str, help='the negligence mode.', default="test")
parser.add_argument('--nth', type=str, help='the nth epoch', default="0_0")
args = parser.parse_args()

access_key_ID = 'AKIASLPL5SQ3PSQE3OZR'
secret_access_key = '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'
s3_bucket_name = 'aw-sumo-cosim-log'
s3 = boto3.resource(
    service_name='s3',
    region_name='us-east-2',
    aws_access_key_id=access_key_ID,
    aws_secret_access_key=secret_access_key
)

log_dir=f"{args.dir}/{args.mode}/raw_data"

# Upload final state
final_state_folder = f"{log_dir}/final_state"
for file in glob.glob(final_state_folder+"/*"):
    new_file_name = file.replace(args.dir,"output_ITE_autoware_aws")
    s3.Bucket(s3_bucket_name).upload_file(Filename=file, Key=new_file_name)

# Upload raw data
raw_data_folder = f"{log_dir}/{args.mode}_{args.nth}"
for file in glob.glob(raw_data_folder+"/*"):
    new_file_name = file.replace(args.dir,"output_ITE_autoware_aws")
    s3.Bucket(s3_bucket_name).upload_file(Filename=file, Key=new_file_name)
    os.remove(file) # delete the file after uploading

# Upload maneuver_challenges
maneuver_challenges_folder = f"{log_dir}/maneuver_challenges"
for file in glob.glob(maneuver_challenges_folder+"/*"):
    new_file_name = file.replace(args.dir,"output_ITE_autoware_aws")
    s3.Bucket(s3_bucket_name).upload_file(Filename=file, Key=new_file_name)
