// Base
#include "robo_trace_openssl_plugins/encryption/partial/stage.hpp"
// Std
#include <algorithm>
// OpenSSL
#include <openssl/rand.h>


namespace robo_trace {

OpenSSLPartialEncryptionProcessingStage::OpenSSLPartialEncryptionProcessingStage(const OpenSSLPartialEncryptionConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const std::string& message_type) 
: ProcessingStage(ProcessingStage::Mode::FORWARD, "openssl_partial_encryption"), m_configuration(configuration), m_key_manager(key_manager) {

    /*
        Initialize EVP related members.
    */

    m_encryption_method = EVP_get_cipherbyname(m_configuration->getEncryptionMethod().c_str());
    m_encryption_context = EVP_CIPHER_CTX_new();

    /*
        Create random symetric encryption key.
    */
    
    // 256bit key. Maybe use less if to much overhead.
    m_key.resize(32);
    
    if (!RAND_bytes((unsigned char*) &m_key[0], 32)) {
        // TODO: Error
        return;
    }

    // TODO: Write back Key for decryption.

    /*
        Setup the IV vector.
    */

    // For AES 256 CBC the IV size is 16 bytes (128 bits).
    // TODO: Make this dynamic
    m_iv.resize(16);

    /*
        Fetch the message template
    */

    m_msg_template = message_description_provider->getMessageDescription(message_type)->message_template;
    ROS_INFO_STREAM("Fetched template for: " << message_type << " -> " << m_msg_template);

    /*
        Fetch the message description
    */

    std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>& enrcyption_trees = m_configuration->getEncryptionTargetsTree();

    std::string message_type_adjusted = message_type;
    std::replace(message_type_adjusted.begin(), message_type_adjusted.end(), '/', '-');

    std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>::const_iterator matches = enrcyption_trees.find(message_type_adjusted);    

    if (matches == enrcyption_trees.end()) {
        // Error!
        return;
    }    

    m_encryption_tree = matches->second;
    ROS_INFO_STREAM("Encryption Tree: " << m_encryption_tree);

}

OpenSSLPartialEncryptionProcessingStage::~OpenSSLPartialEncryptionProcessingStage() = default;


const OpenSSLPartialEncryptionConfiguration::Ptr OpenSSLPartialEncryptionProcessingStage::getConfiguration() const {
    return m_configuration;
}

const OpenSSLPluginKeyManager::Ptr OpenSSLPartialEncryptionProcessingStage::getKeyManager() const {
    return m_key_manager;
}

     
void OpenSSLPartialEncryptionProcessingStage::process(MessageProcessingContext::Ptr& context) {

    // Sample IV for this encryption process.
    if (!RAND_bytes((unsigned char*) &m_iv[0], 16)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Could not sample random IV for message encryption.");
        return;
    }

    context->getMetadata()->append("iv", m_iv.c_str(), 16);

    mongo::BSONObjBuilder builder;
    size_t bytes_read = 0;

    // Starting with DoDeserialize = true
    serialize<true>(m_msg_template, builder, context->getMessage()->getStreamData(), bytes_read, m_encryption_tree);

    context->getMessage()->setSerialized(builder.obj());
   
}

template<bool DoSerialization>
void OpenSSLPartialEncryptionProcessingStage::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, mongo::BSONObjBuilder& builder, const uint8_t* stream, size_t& bytes_read, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr encryption_tree) {
   
    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];
        const std::string& sub_template_name = msg_template->compound.names[idx];
        
        // May need to encrypt this message.
        if (DoSerialization && encryption_tree != nullptr) {
           
            // Is the current sub template a target? Yes? Encrypt!
            if(std::find(encryption_tree->targets.begin(), encryption_tree->targets.end(), sub_template_name) != encryption_tree->targets.end()) {

                // Currently only non authenticated methods. GCM and CCM probably not relevant anyways.
                if(!EVP_EncryptInit_ex(m_encryption_context, EVP_aes_256_cbc(), NULL, (unsigned char*) &m_key[0], (unsigned char*) &m_iv[0])) {
                    // TODO: Error
                    return;
                }

                // We need to know how many byte to encrypt and also advance the read bytes index!
                size_t bytes_read_encrypted_block = bytes_read;
                serialize<false>(sub_template, builder, stream, bytes_read_encrypted_block, nullptr);
                
                // How many bytes are there to be encrypted?
                const int encrypted_block_length = (int) bytes_read_encrypted_block - bytes_read;
                // Possible padding
                const int block_size = EVP_CIPHER_CTX_block_size(m_encryption_context);
                // The ciper text may be longer than the original text due to possible padding.
                const int cipher_max_legth = encrypted_block_length + block_size;

                // No way arround allocating a new chunk on the heap here. :/
                std::unique_ptr<uint8_t[]> cipher_text(static_cast<uint8_t*>(malloc(cipher_max_legth)));
                int cipher_text_length = 0;

                if(!EVP_EncryptUpdate(m_encryption_context, cipher_text.get(), &cipher_text_length, stream, encrypted_block_length)) {
                    // TODO: Error
                    return;
                }

                int length = 0;
                // Usually does not add anything to the cipher text, but might i.e. for padding.
                if(!EVP_EncryptFinal_ex(m_encryption_context, cipher_text.get() + cipher_text_length, &length)) {
                    // TODO: Error
                    return;
                }

                builder.appendBinData(sub_template_name, cipher_max_legth, mongo::BinDataType::BinDataGeneral, cipher_text.get());
            
                bytes_read = bytes_read_encrypted_block;
                continue;
            }
        }

        // https://github.com/StefanFabian/ros_babel_fish/blob/kinetic/ros_babel_fish/include/ros_babel_fish/generation/message_template.h
        switch (sub_template->type) {
            case ros_babel_fish::MessageTypes::Compound: {
                
                if (DoSerialization) {
                    
                    
                    OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;
                    
                    if (encryption_tree != nullptr) {

                        std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(sub_template_name);

                        // No targets under this component.
                        if (matches != encryption_tree->children.end()) {
                            sub_encryption_tree = matches->second;
                        }

                    }

                    mongo::BufBuilder& sub_buff_builder = builder.subobjStart(sub_template_name);
                    mongo::BSONObjBuilder sub_obj_builder(sub_buff_builder);

                    serialize<true>(sub_template, sub_obj_builder, stream, bytes_read, sub_encryption_tree);

                    sub_obj_builder.done();
                } else {
                    // No need to create a sub builder as stuff is not appended anyways
                    serialize<false>(sub_template, builder, stream, bytes_read, nullptr);
                }

                break;
            }
            case ros_babel_fish::MessageTypes::Bool: {
                
                if (DoSerialization) {
                    uint8_t val = *reinterpret_cast<const uint8_t*>(stream + bytes_read);
                    builder.appendBool(sub_template_name, val != 0);
                }

                ++bytes_read;
                break;
            }
            case ros_babel_fish::MessageTypes::UInt8:{
                
                if (DoSerialization) {
                    uint8_t value = *reinterpret_cast<const uint8_t*>(stream + bytes_read);
                    // TODO: Maybe cast to unisgned here
                    builder.append(sub_template_name, value);
                }

                bytes_read += sizeof(uint8_t);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt16: {
                
                if (DoSerialization) {
                    uint16_t value = *reinterpret_cast<const uint16_t*>(stream + bytes_read);
                    // TODO: Maybe cast to unisgned here
                    builder.append(sub_template_name, value);
                }

                bytes_read += sizeof(uint16_t);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt32: {
                
                if (DoSerialization) {
                    uint32_t value = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                    // TODO: Maybe cast to unisgned here
                    builder.append(sub_template_name, value);
                }

                bytes_read += sizeof(uint32_t);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt64: {

                if (DoSerialization) {
                    uint64_t value = *reinterpret_cast<const uint64_t*>(stream + bytes_read);
                    // TODO: Maybe cast to unisgned here
                    builder.appendIntOrLL(sub_template_name, value);
                }

                bytes_read += sizeof(uint64_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Int8: {
                
                if (DoSerialization) {
                    int8_t value = *reinterpret_cast<const int8_t*>(stream + bytes_read);
                    builder.append(sub_template_name, static_cast<int>(value));
                }

                bytes_read += sizeof(int8_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Int16: {
                
                if (DoSerialization) {
                    int16_t value = *reinterpret_cast<const int16_t*>(stream + bytes_read);
                    builder.append(sub_template_name, static_cast<int>(value));
                }

                bytes_read += sizeof(int16_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Int32: {

                if (DoSerialization) {
                    int32_t value = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                    builder.append(sub_template_name, static_cast<int>(value));
                }

                bytes_read += sizeof(int32_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Int64: {
                
                if (DoSerialization) {
                    int64_t value = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                    builder.appendIntOrLL(sub_template_name, value);
                }

                bytes_read += sizeof(int32_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Float32: {
                
                if (DoSerialization) {
                    float value = *reinterpret_cast<const float*>(stream + bytes_read);
                    // Mongo has no specialization for float32
                    builder.append(sub_template_name, static_cast<double>(value));
                }

                bytes_read += sizeof(float);
                break;
            }
            case ros_babel_fish::MessageTypes::Float64: {
                
                if (DoSerialization) {
                    double value = *reinterpret_cast<const double*>(stream + bytes_read);
                    builder.append(sub_template_name, value);
                }

                bytes_read += sizeof(double);
                break;
            }
            case ros_babel_fish::MessageTypes::String: {

                const uint8_t *begin = stream + bytes_read;
                const uint32_t length = *reinterpret_cast<const uint32_t*>(begin);
                
                if (DoSerialization) {
                    builder.append(sub_template_name, std::string(reinterpret_cast<const char*>(begin + 4), length));
                }

                bytes_read += length + sizeof(uint32_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Time: {
                
                if (DoSerialization) {
                    uint32_t secs = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                    uint32_t nsecs = *reinterpret_cast<const uint32_t*>(stream + bytes_read + 4);
                    // Serialize time as a nested object.
                    builder.append(sub_template_name, BSON("secs" << secs << "nsecs" << nsecs));
                }

                bytes_read += 8;
                break;
            }
            case ros_babel_fish::MessageTypes::Duration: {
                
                if (DoSerialization) {
                    int32_t secs = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                    int32_t nsecs = *reinterpret_cast<const int32_t*>(stream + bytes_read + sizeof( int32_t ));
                    // Serialize duration as a nested object.
                    builder.append(sub_template_name, BSON("secs" << secs << "nsecs" << nsecs));
                }

                bytes_read += 8;
                break;
            }
            // TODO: Handle other cases
            case ros_babel_fish::MessageTypes::Array: {
 
                ssize_t length = sub_template->array.length;
                
                bool fixed_length = length >= 0;
                //stream += bytes_read;
                
                if (!fixed_length) {     
                    length = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                    // stream += sizeof( uint32_t );
                    bytes_read += sizeof(uint32_t);
                }

                if (length == 0) {
                    continue;
                }

                // The buffer will point to empty if in counting mode
                mongo::BSONArrayBuilder sub_array_builder = DoSerialization ? mongo::BSONArrayBuilder(builder.subarrayStart(sub_template_name)) : mongo::BSONArrayBuilder();
 
                switch ( sub_template->array.element_type) {
                    case ros_babel_fish::MessageTypes::Bool: {

                        if (DoSerialization) {
                            for (size_t idx = 0; idx < length; ++idx) {
                                sub_array_builder.appendBool(*(stream + bytes_read + idx) != 0);
                            }
                        }

                        bytes_read += sizeof(uint8_t) * length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt8: {
                        
                        size_t array_byte_length = sizeof(uint8_t) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt16: {

                        size_t array_byte_length = sizeof(uint16_t) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt32: {

                        size_t array_byte_length = sizeof(uint32_t) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt64: {

                        size_t array_byte_length = sizeof(uint64_t) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int8: {
                        
                        size_t array_byte_length = sizeof(int8_t) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;;
                    }
                    case ros_babel_fish::MessageTypes::Int16: {

                        size_t array_byte_length = sizeof(int16_t) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int32: {

                        size_t array_byte_length = sizeof(int32_t) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int64: {

                        size_t array_byte_length = sizeof(int64_t) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float32: {

                        size_t array_byte_length = sizeof(float) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float64: {

                        size_t array_byte_length = sizeof(double) * length;

                        if (DoSerialization) {
                            builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                        }

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::String: {

                        size_t offset = 0;

                        for (ssize_t idx = 0; idx < length; ++idx){

                            uint32_t str_length = *reinterpret_cast<const uint32_t*>(stream + bytes_read + offset);
                            offset +=  sizeof(uint32_t);

                            if (DoSerialization) {
                                sub_array_builder.append(std::string(reinterpret_cast<const char*>(stream + bytes_read + offset), str_length));
                            }
                            
                            offset += str_length;
                           
                        }

                        bytes_read += offset;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Time: {
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Duration: {
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Compound: {
                        for (ssize_t idx = 0; idx < length; ++idx) {
                            
                            // Same as with unary comound: Fetch encryption tree if in deserialize otherwise just count
                            if (DoSerialization) {
               
                                OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;
                        
                                if (encryption_tree != nullptr) {

                                    std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(sub_template_name);

                                    // No targets under this component.
                                    if (matches != encryption_tree->children.end()) {
                                        sub_encryption_tree = matches->second;
                                    }

                                }

                                mongo::BufBuilder& sub_obj_buff_builder = sub_array_builder.subobjStart();
                                mongo::BSONObjBuilder sub_obj_builder(sub_obj_buff_builder);

                                serialize<true>(sub_template->array.element_template, sub_obj_builder, stream, bytes_read, sub_encryption_tree);

                                sub_obj_builder.done();

                            } else {
                                serialize<false>(sub_template->array.element_template, builder, stream, bytes_read, nullptr);
                            }

                        }
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Array:
                    case ros_babel_fish::MessageTypes::None:
                        // These don't exist here
                        break;
                }

                if (DoSerialization) {
                    sub_array_builder.done();
                }
                
                break;
            }
            case ros_babel_fish::MessageTypes::None:
                break;
        }
    }

}

}