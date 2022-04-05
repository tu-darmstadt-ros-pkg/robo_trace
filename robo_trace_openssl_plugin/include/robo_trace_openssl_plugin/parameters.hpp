#pragma once

// Whether to offload encrypted binary bload into gridfs buckets. 
// NOTE: This is a work in progress and currently, only serialization is implemented.
// #define MODULE_PARTIAL_ENCRYPTION_OFFLOAD_ENCRYPTED_BLOBS

// If enabled, the forward hash chain module will synchronize access to shared resources.
// #define MODULE_HASH_CHAIN_FORWARD_CONCURRENT
// If enabled, a single OpenSSL will allocated and reused for the forward hash chain module. 
#define MODULE_HASH_CHAIN_FORWARD_REUSE_CONTEXT
// If enabled, a hashing squence number will be added to each message. 
// #define MODULE_HASH_CHAIN_SEQUENCE_NUMBER_ENABLE
// The key for storing the sequence number under.
#define MODULE_HASH_CHAIN_SEQUENCE_NUMBER_KEY "hash_sequence_number"

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