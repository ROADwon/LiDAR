import argparse
import open3d as o3d
import numpy as np
import os

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--src_path', required= True)
    parser.add_argument('--dest_path', required=True)
    parser.add_argument('--size_float', default=4, type=int)
    args = parser.parse_args()
    
    return args

if __name__ =="__main__":
    print("데이터 경로 / 데이터 저장 경로 입력")
    args = get_args()
    
    if os.path.exist(args.dest_path) == Fasle:
        os.makedirs(args.dest_path)
   
   file_to_open = os.listdir()     
    
    