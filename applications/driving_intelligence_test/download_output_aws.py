import boto3
from tqdm import tqdm
import os

access_key_ID = 'AKIASLPL5SQ3PSQE3OZR'
secret_access_key = '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'
s3_bucket_name = 'aw-sumo-cosim-log'
s3 = boto3.resource(
    service_name='s3',
    region_name='us-east-2',
    aws_access_key_id=access_key_ID,
    aws_secret_access_key=secret_access_key
)

remote_folder = "output_ITE_autoware_universe_aws"

local_folder = "output/"+"cosim_test_aws_v5"
os.makedirs(local_folder, exist_ok=True)
def downloads3(obj_key):
    print("obj_key: ", obj_key)
    print()
    output_file_path = obj_key.replace(remote_folder,local_folder)
    output_file_name = output_file_path.split("/")[-1]
    output_file_folder = output_file_path.replace(output_file_name,"")
    os.makedirs(output_file_folder, exist_ok=True)
    s3.Bucket(s3_bucket_name).download_file(obj_key, output_file_path)

filter_keys = [remote_folder,"cosim_test_aws_v5"]
obj_key_list = []
for obj in tqdm(s3.Bucket(s3_bucket_name).objects.all()):
    # filter files
    filter_flag = False
    for k in filter_keys:
        if k not in obj.key:
            filter_flag = True
    
    if not filter_flag:
        obj_key_list.append(obj.key)
print(len(obj_key_list))


from multiprocessing import Pool
with Pool(5) as p:
    for _ in tqdm(p.imap_unordered(downloads3, obj_key_list), total=len(obj_key_list)):
        pass
