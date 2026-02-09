FROM ros:noetic

# 设置工作目录
WORKDIR /workspace

# 配置ROS环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
