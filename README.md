# lazyros

A simple and friendly terminal UI for ROS2.

![image](./asset/lazyros_usage_short_movie.gif)



## Features

### Node (include life-cycle node)

- viewing logs by node
- viewing node info
- starting / stopping node
- changing life-cycle node status

### Topic

- viewing topic (like a `ros2 topic echo`)

### Parameter

- setting / viewing parameters



## Installation

- pip (recommended)
  ```shell
  pip install lazyros
  ```

- from source
  ```shell
  git clone https://github.com/TechMagicKK/lazyros.git
  cd lazyros
  pip install -r requirements.txt
  ```



## Usage

Just run following command.

```shell
lazyros
```



## Getting started

1. Run talker demo node
   ```shell
   ros2 run demo_nodes_cpp talker
   ```

2. Run listener demo node
   ```
   ros2 run demo_nodes_cpp listener
   ```

3. Run lazyros
   ```shell
   lazyros
   ```



## Help

- Trouble
  If you encounter trouble, please feel free to open an issue!

- Request

  Any request is welcome, please feel free to request in issue!



## Release

Check [How to release](./docs/release.md).
