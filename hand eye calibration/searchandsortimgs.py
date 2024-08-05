import os
import shutil
import string
from itertools import product

def generate_letter_pairs():
    letters = string.ascii_lowercase
    return [''.join(pair) for pair in product(letters, repeat=2)]

def process_numeric_subdirectories(directory):
    # Generate letter pairs from 'aa' to 'zz'
    letter_pairs = generate_letter_pairs()
    
    # Get a list of all subdirectories in the specified directory
    subdirectories = [name for name in os.listdir(directory) if os.path.isdir(os.path.join(directory, name))]
    
    # Filter to only numeric subdirectories
    numeric_subdirectories = [subdir for subdir in subdirectories if subdir.isdigit()]
    
    # Sort subdirectories to ensure consistent ordering
    numeric_subdirectories.sort(key=int)
    
    # Process each numeric subdirectory
    for index, subdir in enumerate(numeric_subdirectories):
        if index >= len(letter_pairs):
            print("Error: More numeric subdirectories than letter pairs.")
            break
        
        letter_pair = letter_pairs[index]
        source_folder = os.path.join(directory, subdir, '1', 'Scanimages_00')
        target_folder = os.path.join(directory, 'lettered_images')
        
        # Create the target folder if it doesn't exist
        os.makedirs(target_folder, exist_ok=True)
        
        # Find Image_00.bmp in source_folder and copy to target_folder with new name
        image_source = os.path.join(source_folder, 'Image_00.bmp')
        image_dest = os.path.join(target_folder, f'{letter_pair}.bmp')
        
        # Copy Image_00.bmp to target_folder with a new name based on letter pair
        if os.path.exists(image_source):
            shutil.copy(image_source, image_dest)
            print(f"Processed folder {subdir}: Copied Image_00.bmp to {image_dest}")
        else:
            print(f"Image_00.bmp not found in {source_folder}.")

# Example usage:
directory_to_search = r''  # Replace with your directory path
process_numeric_subdirectories(directory_to_search)
