#!/usr/bin/python3.5
# -*- coding: utf-8 -*-

"""
  Voxnet evaluation
"""

import tensorflow as tf

from nets.voxNet import VoxNet
import utils.data_helper as helper

FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_string(
    'model_dir', './logs',
    """directory for training model checkpoint.""")
tf.app.flags.DEFINE_string(
    'npy_dir', './data/datasets/npy_generated',
    """directory to the preprocess training dataset.""")

def eval(argv):
    """
    Print the predicted class and ground truth class.

    Args:
    `argv`:['./',model_dir,data_folder] a tuple of args.
    `model_dir`:the folder of trained model
    `data_folder`:the folder of `*.npy` data to be inferred
    """

    model = VoxNet()

    # Voxnet Estimator: model init
    voxel_classifier = tf.estimator.Estimator(model_fn=model.core, model_dir=FLAGS.model_dir)

    # Evaluating data collector
    voxels, labels = helper.load_data_from_npy(FLAGS.npy_dir, mode='testing')

    print('Start testing...')

    # Evaluate the model and print results
    eval_input_fn = tf.estimator.inputs.numpy_input_fn(
        x={"OccuGrid_input": voxels},
        y=labels,
        num_epochs=1,
        shuffle=False
    )

    # results = voxel_classifier.evaluate(input_fn=eval_input_fn)
    # Get predictions
    predictions = voxel_classifier.predict(input_fn=eval_input_fn)

    # Print results
    top_k = 3
    for dict_predict, gt in zip(predictions, labels):
        print ('Predicted: {}, Ground Truth: {}'.format(
            helper.get_SUOD_label(dict_predict['pred_cls']), helper.get_SUOD_label(gt)
        ))
        # Top K label, k=3
        """
        Code sample of top k index
        In [1]: import numpy as np
        In [2]: arr = np.array([1, 3, 2, 4, 5])
        In [3]: arr.argsort()[-3:][::-1]
        Out[3]: array([4, 3, 1])...
        """
        arr = dict_predict['probabilities']
        idx = arr.argsort()[-top_k:][::-1]
        print('Top {} labels: {} {} {}...'.format(
            top_k, helper.get_SUOD_label(idx[0]), helper.get_SUOD_label(idx[1]), helper.get_SUOD_label(idx[2])
        ))

    voxel_classifier.evaluate(input_fn=eval_input_fn)

    print('Finished testing.')

if __name__ == '__main__':
    tf.app.run(main=eval, argv=[])