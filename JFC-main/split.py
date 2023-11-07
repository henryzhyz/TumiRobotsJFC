"""
A simple program split the point cloud data into one of the 5 categories.

sys.argv[1]: the file of pre-processing tunnel data
output: Corresponding category files.
"""



import sys
import os

input = open(sys.argv[1], "r")
file_num = sys.argv[1].split("_")[3][0]

try:
    os.mkdir("/users/ron_ming/desktop/Area_1/tunnel_{0}/Annotations".format(file_num))
except OSError:
    print ("Creation of the directory failed")

output_rooft = open("/users/ron_ming/desktop/Area_1/tunnel_{0}/Annotations/rooftbolt_1.txt".format(file_num), "w")
output_wall = open("/users/ron_ming/desktop/Area_1/tunnel_{0}/Annotations/wall_1.txt".format(file_num), "w")
output_ceiling = open("/users/ron_ming/desktop/Area_1/tunnel_{0}/Annotations/ceiling_1.txt".format(file_num), "w")
output_floor = open("/users/ron_ming/desktop/Area_1/tunnel_{0}/Annotations/floor_1.txt".format(file_num), "w")
output_other = open("/users/ron_ming/desktop/Area_1/tunnel_{0}/Annotations/other_1.txt".format(file_num), "w")


line = input.readline()
isFirst = True

while line:
    if isFirst:
        isFirst = False
        line = input.readline()
        continue
    
    previous_row = line.strip("\n")
    elements = previous_row.split() 
    new_row = [*elements[0:3], *elements[3:6]]
    output_row = " ".join(new_row) + "\n"

    
    if elements[6] == "1": ## rooftbolt
        output_rooft.write(output_row)
    elif elements[6] == "2": ## ceiling
        output_ceiling.write(output_row)
    elif elements[6] == "3": ## wall
        output_wall.write(output_row)
    elif elements[6] == "4": ## floor
        output_floor.write(output_row)
    else: ## cables/other
        output_other.write(output_row)
    line = input.readline()

input.close()
output_wall.close()
output_rooft.close()
output_floor.close()
output_other.close()
output_ceiling.close()
    