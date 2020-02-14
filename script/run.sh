#data path
input_points_file=./data/polygon.txt;
plane_coefficients_file=./data/plane_coefficients.txt;
output_points_file=./data/projected_polygon.txt

./bin/project2plane ${input_points_file} ${plane_coefficients_file} ${output_points_file}