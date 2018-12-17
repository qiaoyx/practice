import tensorflow as tf

a = tf.constant([[1, 2, 3], [4, 5, 6], [7, 8, 9]], name='a')
b = tf.constant([[1, 2, 3], [4, 5, 6], [7, 8, 9]], name='b')
add_op = a * b

with tf.Session() as session:
    print(session.run(add_op))