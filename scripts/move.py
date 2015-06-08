
import glob
import os
import shutil


diff = lambda x: [x[i + 1] - x[i] for i in range(len(x[:-1]))]

if __name__ == '__main__':
    files = glob.glob("*.CR2")
    files = sorted(files)
    timestamps_str = (x[7:13] for x in files)
    timestamps_int = (int(x) for x in timestamps_str if x.isdigit())
    time_delta = diff(list(timestamps_int))
    groups = [True] + [x > 100 for x in time_delta]
    current_group = None
    for idx, file_ in enumerate(files):
        if groups[idx]:
            current_group = files[idx]
            current_group_path = current_group.replace("_","/").split(".")[0]
            try:
                os.makedirs(current_group_path)
            except FileExistsError:
                pass
        shutil.move(file_, os.path.join(current_group_path, file_))
