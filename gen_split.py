import os
import argparse
import numpy as np
import sklearn

save_folder = '/1500_samples'

def gen_split(base_folder, N, val):
    all = np.arange(N)
    val_samples = round(N*val)
    train_samples = N - val_samples
    mask = np.zeros(N,dtype=bool)
    mask[np.random.choice(N, val_samples)] = True
    val = all[mask]
    train = all[~mask]

    test = all 
    trainval = all
    
    train_file = base_folder+"train.txt"
    val_file = base_folder+"val.txt"
    test_file = base_folder+"test.txt"
    trainval_file = base_folder+"trainval.txt"

    f=open(train_file, 'w')
    np.savetxt(f,train.astype(int),fmt='%06i')
    f.close()

    f=open(val_file, 'w')
    np.savetxt(f,val.astype(int),fmt='%06i')
    f.close()

    f=open(test_file, 'w')
    np.savetxt(f,test.astype(int),fmt='%06i')
    f.close()

    f=open(trainval_file, 'w')
    np.savetxt(f,trainval.astype(int),fmt='%06i')
    f.close()

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Generate split files")
    # arg_parser.add_argument("split_folder", type=str, help="folder to output split files to")   
    arg_parser.add_argument("N", type=int, help="total number of data samples")  
    arg_parser.add_argument("val", type=float, help="val proportion in train/val")   
    args = arg_parser.parse_args() 

    os.chdir('/home/navlab-admin/AirSim-MOT/data')
    datapath = os.getcwd()
    # split_folder = args.split_folder
    base_folder = datapath + save_folder + '/ImageSets/'

    gen_split(base_folder, args.N, args.val)

