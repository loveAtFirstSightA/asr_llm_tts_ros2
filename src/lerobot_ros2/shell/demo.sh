#!/bin/bash 

rm -rf /home/kc/.cache/huggingface/lerobot/eval_actm

/home/kc/anaconda3/envs/lerobot2/bin/python /home/kc/lerobot_voice/lerobot/scripts/control_robot.py \
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
