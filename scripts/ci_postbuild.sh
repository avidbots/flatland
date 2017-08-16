

# cd into catkin workspace
# catkin clean -y
# catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cd /root/catkin_ws
run-clang-tidy-3.8.py -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_server
run-clang-tidy-3.8.py -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_plugins
run-clang-tidy-3.8.py -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_viz