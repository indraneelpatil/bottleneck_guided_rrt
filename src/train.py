#!/usr/bin/python3.5
# -*- coding: utf-8 -*-

"""
  Voxnet training
"""

import tensorflow as tf

from nets.voxNet import VoxNet
import utils.data_helper as helper

FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_string(
    'log_dir', './logs/',
    """Directory for training logs, including training summaries as well as training model checkpoint.""")
tf.app.flags.DEFINE_string(
    'npy_dir', './data/datasets/npy_generated',
    """directory to the preprocess training dataset.""")
tf.app.flags.DEFINE_bool(
    'clear_log', False,
    """force to clear old logs if exist.""")

tf.app.flags.DEFINE_integer(
    'num_epochs', 8,
    """The numbers of epochs for training, train over the dataset about 8 times.""")
tf.app.flags.DEFINE_integer(
    'batch_size', 32,
    """The numbers of training examples present in a single batch for every training.""")

def train(argv=None):
    model = VoxNet()

    # Voxnet Estimator: model init
    voxel_classifier = tf.estimator.Estimator(model_fn=model.core, model_dir=FLAGS.log_dir)

    # Trainning data collector
    voxels, labels = helper.load_data_from_npy(FLAGS.npy_dir, mode='training')

    # Set up logging for predictions
    log = {"probabilities": "softmax_tensor"}
    logging_hook = tf.train.LoggingTensorHook(tensors=log, every_n_iter=10)

    # Train the model
    train_input_fn = tf.estimator.inputs.numpy_input_fn(
        x={"OccuGrid_input": voxels},
        y=labels,
        batch_size=FLAGS.batch_size,
        num_epochs=FLAGS.num_epochs,
        shuffle=True
    )

    print("You can use Tensorboard to visualize the results by command 'tensorboard --logdir={}'.".format(FLAGS.log_dir))
    input("Press Enter to Start training...")

    voxel_classifier.train(
        input_fn=train_input_fn,
        steps=5000,
        hooks=[logging_hook]
    )
    print('Finished training.')

if __name__ == '__main__':
    # delete old logs
    if FLAGS.clear_log:
        if tf.gfile.Exists(FLAGS.log_dir):
            tf.gfile.DeleteRecursively(FLAGS.log_dir)
        tf.gfile.MakeDirs(FLAGS.log_dir)

    tf.app.run(main=train, argv=[])