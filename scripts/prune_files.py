import os
import shutil

def copy_every_nth_image(input_folder, output_folder, n=10):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Get list of files in input folder
    files = os.listdir(input_folder)

    # Iterate over files and copy every nth file
    for i, file_name in enumerate(files):
        if i % n == 0:
            shutil.copy(os.path.join(input_folder, file_name), output_folder)

if __name__ == '__main__':
    input_folder = 'C:\Users\Adam\Documents\AirSim\2024-03-14-22-17-32\images'
    output_folder = 'C:\Users\Adam\Documents\AirSim\2024-03-14-22-17-32\images_10'
    copy_every_nth_image(input_folder, output_folder, n=100)
