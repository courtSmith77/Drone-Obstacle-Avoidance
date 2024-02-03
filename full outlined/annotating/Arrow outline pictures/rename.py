import os

dir = "down"
num_batches = 2

count = 0
for i in range(num_batches):
    folder_path = os.path.join(".", dir, f"batch{i+1}")
    for image_file in os.listdir(folder_path):
        new_filename = dir + '_' + str(count) + '.jpg'
        new_path = os.path.join(".", "all", new_filename)
        old_path = os.path.join(folder_path, image_file)
        
        os.rename(old_path, new_path)

        count+=1

# count = 0
# for i in range(num_batches):
#     folder_path = os.path.join(".", dir)
#     for image_file in os.listdir(folder_path):
#         new_filename = dir + '_' + str(count) + '.jpg'
#         new_path = os.path.join(folder_path, new_filename)
#         old_path = os.path.join(folder_path, image_file)
        
#         os.rename(old_path, new_path)

#         count+=1