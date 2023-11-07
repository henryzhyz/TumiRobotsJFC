# JFC

Run split.py to separate the tunnel point cloud data into 5 different categories. A example of how to execute splity.py is shown below:

    python split.py <file_path>

Run the collect_indoor3d_data.py to change the data format from .txt to .npy

    cd data_utils
    python collect_indoor3d_data.py

**Note**: Before run the above command, make sure the file directory is corrected both in indoor3d.py and collect_indoor3d_util.py

To use pre-trained model change the block of the code in the train_semseg.py

    line130: toch.load('file_path') 'Directory for the checkpoint that save in log folder'
    line131: # classifier.conv2 = torch.nn.Conv1d(128, 13, 1) (uncomment this line)
    line135, 136: uncomment those two line

Trained the model in MSG mode: change the config setting in train_semseg.py

    line38: change pointnet2_sem_seg to pointnet2_sem_seg_msg