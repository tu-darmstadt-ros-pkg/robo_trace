# RoboTrace
This repository contains the source for RoboTrace .

## Dependencies
For in order to use RoboTrace, make sure you a-priori have OpenSSL and MongoDB installed. For the later one refear to https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/, if you have not installed it already. Make sure to also have Ros Babel Fish (https://github.com/StefanFabian/ros_babel_fish) in your ros build path, as this package is essential for RoboTrace to work.

## Configuration
RoboTrace in its core is a tool to persist ROS message streams. As such, its main functionality is to transform messages and insert them into a database. Naturally the database choice has been locked to MongoDB. The transforms that are being applied to a message however are fully configurable. Refear to the example configuration to get an overview on how this might look like. Currently there are several different types of transforms that are already implemented. Namely these are:

 - openssl_hash_chain: Builds up a hash chain by hashing the current message with the hash of the previous message. This might protect against insertion attacks onto the final log.
 - openssl_signature: Signs the message by the means of a randomly generated asymetric keypair. 
