```bash
python -m colcon build --symlink-install   --cmake-args -DPython3_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"
```