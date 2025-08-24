import os
import pandas as pd
import shutil
datafolder = [el for el in os.listdir() if (el.startswith("P3") and not os.path.isfile(el))]

new_data_folder = "P3CompleteData"
os.mkdir(new_data_folder)
new_img_path = os.path.join(new_data_folder, "IMG")
os.mkdir(new_img_path)
frames = []
for el in datafolder:
    src_img_folder = os.path.join(el, "IMG")
    for img in os.listdir(src_img_folder):
        shutil.copy(os.path.join(src_img_folder, img), os.path.join(new_img_path, img))
    frames.append(pd.read_csv(os.path.join(el, "driving_log.csv"),
        names=["center", "left", "right", "steering", "throttle", "break", "speed"]))

final_df = pd.concat(frames)
final_df.to_csv(os.path.join(new_data_folder, 'driving_log.csv'), header=False, index=False)
print(final_df.shape[0]*3, len(os.listdir(new_img_path)))
