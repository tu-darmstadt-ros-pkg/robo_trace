#pragma once


// Whether to check if all bytes have been consumed after comined encryption and serialization
// #define MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES


// ############# Ros param related definitions #############

#define ROS_PARAM_HASH_CHAIN_METHOD_NAME "hashing_method"
#define ROS_PARAM_HASH_CHAIN_METHOD_DEFAULT "SHA256"

#define ROS_PARAM_HASH_CHAIN_RESULT_STORAGE_KEY_NAME "result_storage_key"
#define ROS_PARAM_HASH_CHAIN_RESULT_STORAGE_KEY_DEFAULT "hash_chain"

#define ROS_PARAM_SIGNATURE_HASH_METHOD_NAME "hashing_method"
#define ROS_PARAM_SIGNATURE_HASH_METHOD_DEFAULT "SHA128"

#define ROS_PARAM_SIGNATURE_INPUT_STORAGE_KEY_NAME "input_store_key"
#define ROS_PARAM_SIGNATURE_INPUT_STORAGE_KEY_DEFAULT "hash_chain"

#define ROS_PARAM_SIGNATURE_RESULT_STORAGE_KEY_NAME "result_storage_key"
#define ROS_PARAM_SIGNATURE_RESULT_STORAGE_KEY_DEFAULT "signature"

#define ROS_PARAM_ENCRYPTION_METHOD_NAME "encryption_method"
#define ROS_PARAM_ENCRYPTION_METHOD_DEFAULT "aes-256-cbc"

#define ROS_PARAM_ENCRYPTION_TARGET_KEY "encryption_targets"