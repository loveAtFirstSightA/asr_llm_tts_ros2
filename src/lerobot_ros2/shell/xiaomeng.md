python lerobot/common/robot_devices/cameras/opencv.py \
    --images-dir outputs/images_from_opencv_cameras


python lerobot/scripts/find_motors_bus_port.py


python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --robot.cameras='{}' \
  --control.type=teleoperate

python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=teleoperate


sudo chmod 666 /dev/tty*

python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a lego block and put it in the bin." \
  --control.repo_id=./eval_act_so100_test \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=30 \
  --control.num_episodes=10 \
  --control.push_to_hub=false \
  --control.policy.path=050000/pretrained_model



python lerobot/scripts/train.py \
  --dataset.repo_id=so100_testm \
  --policy.type=act \
  --steps=10_000 \
  --save_freq=1_000 \
  --batch_size=10 \
  --output_dir=outputs/train/act_so100_testm \
  --job_name=act_test_dual_cloth \
  --policy.device=cuda \
  --wandb.enable=false


scp -r ./so100 kc@192.168.221.162:/home/kc/workspace/lerobot/.cache/calibration



# 收集数据
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a cloth." \
  --control.repo_id=./so100_03_14_cloth \
  --control.tags='["so100","tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=30 \
  --control.num_episodes=50 \
  --control.push_to_hub=false





python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.repo_id=$./eval_act_so100_test \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=30 \
  --control.num_episodes=10 \
  --control.push_to_hub=true \
  --control.policy.path=outputs/train/act_test/checkpoints/010000/pretrained_model


# ACT:
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a lego block and put it in the bin." \
  --control.repo_id=./eval_actm \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=30 \
  --control.num_episodes=1 \
  --control.push_to_hub=false \
  --control.policy.path=outputs/train/2025-03-18/11-28-20_act/checkpoints/100000/pretrained_model



scp -rP 36653 root@connect.nmb1.seetacloud.com:/root/autodl-tmp/lerobot/outputs/train/diffusion_10_large /home/xiaomeng/code/lerobot/outputs/train
ssh -p 29506 root@connect.nmb1.seetacloud.com



# Diffusion:
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a lego block and put it in the bin." \
  --control.repo_id=./eval_diffusion5 \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30000000 \
  --control.reset_time_s=30 \
  --control.num_episodes=10 \
  --control.push_to_hub=false \
  --control.policy.path=outputs/train/diffusion_ori/004000/pretrained_model


sudo apt install usbtop
sudo apt update$ sudo apt install cmake git libboost-dev libpcap-dev libboost-thread-dev libboost-system-dev
sudo modprobe usbmon
sudo usbtop


python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a lego block and put it in the bin." \
  --control.repo_id=./so100_testm \
  --control.tags='["so100","tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=30 \
  --control.num_episodes=5 \
  --control.push_to_hub=false