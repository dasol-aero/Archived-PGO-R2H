# PGO-R2H

Loop Detection and Pose Graph Optimization

---

## Prerequisite

- **pcl-conversions**

  ```bash
  sudo apt update
  sudo apt install ros-$ROS_DISTRO-pcl-conversions
  ```

- **cv-bridge**

  ```bash
  sudo apt update
  sudo apt install ros-$ROS_DISTRO-cv-bridge
  ```

- [**GTSAM**](https://gtsam.org/get_started/)

  ```bash
  # add PPA
  sudo add-apt-repository ppa:borglab/gtsam-release-4.1
  sudo apt update

  # install
  sudo apt install libgtsam-dev libgtsam-unstable-dev
  ```

---

## Revision History

- **v0.0.1 (Dec 29, 2023)**

  - seems to work properly
  - need to clean up

- **v0.1.0 (Dec 31, 2023)**

  - working properly
  - clear fix tags

---

## Reference

- [**gisbi-kim / SC-A-LOAM**](https://github.com/gisbi-kim/SC-A-LOAM)
- [**yanliang-wang / FAST_LIO_LC**](https://github.com/yanliang-wang/FAST_LIO_LC)
- [**GDUT-Kyle / livox_backend**](https://github.com/GDUT-Kyle/livox_backend)

---
