#!/bin/bash

export DATASET_DIR="./data/datasets/sydney-urban-objects-dataset"
export clear_log=False
export LOG_DIR="./logs"
export num_epochs=8
export batch_size=32

while test $# -gt 0; do
  case "$1" in
    -h|--help)
      echo "Usage: ./scripts/train.sh [options]"
      echo " "
      echo "options:"
      echo "-h, --help           show brief help"
      echo "-dataset_dir         training dataset given point cloud segmentation and corresponding class label, "
      echo "                       default './data/datasets/sydney-urban-objects-dataset'"
      echo "-f                   force to clear old logs if exist"
      echo "-log_dir             path for training logs, including training summaries as well as model parameters,"
      echo "                       default in './logs' and './logs/checkpoint' respectively"
      echo "-num_epochs          the numbers of epochs for training, default train over the dataset about 8 times."
      echo "-batch_size          the numbers of training examples present in a single batch for every training, default 32"
      exit 0
      ;;
    -dataset_dir)
      export DATASET_DIR="$2"
      shift
      shift
      ;;
    -f)
      export clear_log=True
      shift
      ;;
    -log_dir)
      export LOG_DIR="$2"
      shift
      shift
      ;;
    -num_epochs)
      export num_epochs="$2"
      shift
      shift
      ;;
    -batch_size)
      export batch_size="$2"
      shift
      shift
      ;;
    *)
      echo "Usage: ./scripts/train.sh [options]"
      echo " "
      echo "options:"
      echo "-h, --help           show brief help"
      echo "-dataset_dir         training dataset given point cloud segmentation and corresponding class label, "
      echo "                       default './data/datasets/sydney-urban-objects-dataset'"
      echo "-f                   force to clear old logs if exist"
      echo "-log_dir             path for training logs, including training summaries as well as model parameters,"
      echo "                       default in './logs' and './logs/checkpoint' respectively"
      echo "-num_epochs          the numbers of epochs for training, default train over the dataset about 8 times."
      echo "-batch_size          the numbers of training examples present in a single batch for every training, default 32"
      exit 0
      ;;
  esac
done

# prepare training set & testing set
python ./src/preprocess.py \
  --clear_cache
python ./src/preprocess.py \
  --fold 1
python ./src/preprocess.py \
  --fold 2
python ./src/preprocess.py \
  --fold 3 \
  --type testing

python ./src/train.py \
  --dataset_dir $DATASET_DIR \
  --clear_log $clear_log \
  --log_dir $LOG_DIR \
  --num_epochs $num_epochs \
  --batch_size $batch_size