import numpy as np

import tensorflow as tf
from keras.models import load_model


class TLClassifier(object):
    def __init__(self, model_path):
        self.model = load_model(model_path)
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            return np.argmax(self.model.predict(image[None, :, :, :], batch_size=1)[0])
