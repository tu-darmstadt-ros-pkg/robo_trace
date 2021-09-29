// Base
#include "robo_trace_openssl_plugin/encryption/partial/forward.hpp"
// Std
#include <algorithm>
#include <string>
#include <stdexcept>
// OpenSSL
#include <openssl/rand.h>
#include <openssl/rsa.h>
#include <openssl/engine.h>
// Ros
#include <ros/console.h>


#define STAGE_LOGGER_NAME "robo_trace_openssl_partial_encryption_forward"


namespace robo_trace {

OpenSSLPartialEncryptionForwardStage::OpenSSLPartialEncryptionForwardStage(const OpenSSLPartialEncryptionConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const DataContainer::Ptr& metadata) 
: m_configuration(configuration), m_key_manager(key_manager) {
    
    /*
        Initialize EVP related members.
    */

    m_encryption_context = EVP_CIPHER_CTX_new();

    if (m_encryption_context == nullptr) {
        throw std::runtime_error("Could not instantiate new EVP context.");
    }

    /*
        Initialize EVP related members.
    */

    ROS_DEBUG_STREAM_NAMED(STAGE_LOGGER_NAME, "Using encryption method: " << m_configuration->getEncryptionMethod());
    m_encryption_method = EVP_get_cipherbyname(m_configuration->getEncryptionMethod().c_str());
    
    if (m_encryption_method == nullptr) {
        throw std::runtime_error("Encryption method could not be resolved.");
    }

    /*
        Setup the IV vector.
    */

    int iv_length = EVP_CIPHER_iv_length(m_encryption_method);
    m_iv.resize(iv_length);

    /*
        Fetch the message template
    */

    std::string message_type = metadata->getString("message_type");

    ROS_DEBUG_STREAM_NAMED(STAGE_LOGGER_NAME, "Fetching template for: " << message_type);
    ros_babel_fish::MessageDescription::ConstPtr message_description = message_description_provider->getMessageDescription(message_type);
    
    if (message_description == nullptr) {
        throw std::runtime_error("No message description for message found!");
    }
    
    m_msg_template = message_description->message_template;
   
    /*
        Fetch the message description.
    */

    std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>& enrcyption_trees = m_configuration->getEncryptionTargetsTree();

    std::string message_type_adjusted = message_type;
    std::replace(message_type_adjusted.begin(), message_type_adjusted.end(), '/', '-');

    std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>::const_iterator matches = enrcyption_trees.find(message_type_adjusted);    

    if (matches == enrcyption_trees.end()) {
        m_encryption_tree = nullptr;
        //throw std::runtime_error("Could not find encryption tree for message.");
    } else {   
        m_encryption_tree = matches->second;
    }

     /*
        Create random symetric encryption key.
    */
    
    int key_length = EVP_CIPHER_key_length(m_encryption_method);
    m_key.resize(key_length);
    
    if (!RAND_bytes(&m_key[0], key_length)) {
        throw std::runtime_error("Failed generating encryption key.");
    }

    /*
        Save the encrypted key to the container.
    */
    
    evp_pkey_ctx_ptr pkey_context(EVP_PKEY_CTX_new(m_key_manager->getAuthorityPublicKey(), nullptr));
    
    if (EVP_PKEY_encrypt_init(pkey_context.get()) <= 0) {
        throw std::runtime_error("Failed initializing pkey encryption context.");
    }

    if (EVP_PKEY_CTX_set_rsa_padding(pkey_context.get(), RSA_PKCS1_OAEP_PADDING) <= 0) {
        throw std::runtime_error("Failed seeting RSA padding.");
    }

    size_t encrypted_key_size;

    if (EVP_PKEY_encrypt(pkey_context.get(), nullptr, &encrypted_key_size, &m_key[0], m_key.size()) <= 0) {
      throw std::runtime_error("Failed fetching encrypted key size.");
    }

    std::unique_ptr<unsigned char[]> encrypted_key((unsigned char*) malloc(encrypted_key_size));
    
    if (EVP_PKEY_encrypt(pkey_context.get(), &encrypted_key[0], &encrypted_key_size, &m_key[0], m_key.size()) <= 0) {
        throw std::runtime_error("Failed encrypting key.");
    }

    metadata->append("key", encrypted_key.get(), encrypted_key_size);
    
}

OpenSSLPartialEncryptionForwardStage::~OpenSSLPartialEncryptionForwardStage() = default;

ProcessingMode OpenSSLPartialEncryptionForwardStage::getMode() const {
    return ProcessingMode::CAPTURE;
}
     
void OpenSSLPartialEncryptionForwardStage::process(const ProcessingContext::Ptr& context) {

    // Sample IV for this encryption process.
    if (!RAND_bytes(&m_iv[0], m_iv.size())) {
        throw std::runtime_error("Failed sampling IV!");
    }

    context->getMetadata()->append("iv", &m_iv[0], m_iv.size());

    size_t message_stream_length = 0;
    const std::optional<const uint8_t* const> o_message_stream = context->getUnserializedMessage(message_stream_length);

    if (!o_message_stream) {
        throw std::runtime_error("Unserialized message not present.");
    }

    const uint8_t* const message_stream = o_message_stream.value();

    mongo::BSONObjBuilder builder;
    size_t bytes_read = 0;

    // Starting with DoDeserialize = true
    serialize<true>(m_msg_template, builder, message_stream, bytes_read, m_encryption_tree);

    context->setSerializedMessage(builder.obj());
   
}

template<bool DoSerialization>
void OpenSSLPartialEncryptionForwardStage::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, mongo::BSONObjBuilder& builder, const uint8_t* stream, size_t& bytes_read, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr encryption_tree) {
    
    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];
        const std::string& sub_template_name = msg_template->compound.names[idx];
        
        // May need to encrypt this message.
        if (DoSerialization &&
            encryption_tree != nullptr &&
            // Is the current sub template a target? Yes? Encrypt!
            std::find(encryption_tree->targets.begin(), encryption_tree->targets.end(), sub_template_name) != encryption_tree->targets.end()) {

            // Currently only non authenticated methods. GCM and CCM probably not relevant anyways.
            if(!EVP_EncryptInit_ex(m_encryption_context, m_encryption_method, NULL, &m_key[0], &m_iv[0])) {
                throw std::runtime_error("Failed to initialize EVP context!");
            }

            // We need to know how many byte to encrypt and also advance the read bytes index!
            size_t bytes_read_encrypted_block = bytes_read;
            serialize<false>(sub_template, sub_template_name, builder, stream, bytes_read_encrypted_block, nullptr);

            // How many bytes are there to be encrypted?
            const int encrypted_block_length = (int) bytes_read_encrypted_block - bytes_read;
            // Possible padding
            const int block_size = EVP_CIPHER_CTX_block_size(m_encryption_context);
            // The ciper text may be longer than the original text due to possible padding.
            const int cipher_max_legth = encrypted_block_length + block_size;

            /* 
                No way arround possibly allocating a new chunk on the heap here. :/

                We could directly write to the ByteBuffer of the BSONObjBuilder.
                Though for allocating the byte array, we'd need to know the exact
                size of the array beforehand... We only have a upper limit given
                by cipher_max_legth. We can't shrink the buffer again. Hence, 
                we need to dump everything into an intermediate first. :/

            */
            m_cipher_buffer.resize(cipher_max_legth);
            int cipher_text_length = 0;

            if(!EVP_EncryptUpdate(m_encryption_context, &m_cipher_buffer[0], &cipher_text_length, stream + bytes_read, encrypted_block_length)) {
                throw std::runtime_error("Failed feeding message for encryption.");
            }

            int length = 0;
            // Usually does not add anything to the cipher text, but might i.e. for padding.
            if(!EVP_EncryptFinal_ex(m_encryption_context, &m_cipher_buffer[0] + cipher_text_length, &length)) {
                throw std::runtime_error("Failed finalizing encryption.");
            }

            // Arrrg.
            builder.appendBinData(sub_template_name, cipher_text_length + length, mongo::BinDataType::BinDataGeneral, &m_cipher_buffer[0]);
        
            bytes_read = bytes_read_encrypted_block;
                
        } else {
            if (DoSerialization) {
                serialize<true>(sub_template, sub_template_name, builder, stream, bytes_read, encryption_tree);
            } else {
                serialize<false>(sub_template, sub_template_name, builder, stream, bytes_read, encryption_tree);        
            }
        }
    }

}

template<bool DoSerialization>
void OpenSSLPartialEncryptionForwardStage::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const std::string& name, mongo::BSONObjBuilder& builder, const uint8_t* stream, size_t& bytes_read, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr encryption_tree) {

    switch (msg_template->type) {
        case ros_babel_fish::MessageTypes::Compound: {
                
            if (DoSerialization) {     
                
                OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;
                
                if (encryption_tree != nullptr) {

                    std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(name);

                    // No targets under this component.
                    if (matches != encryption_tree->children.end()) {
                        sub_encryption_tree = matches->second;
                    }

                }

                mongo::BufBuilder& sub_buff_builder = builder.subobjStart(name);
                mongo::BSONObjBuilder sub_obj_builder(sub_buff_builder);

                serialize<true>(msg_template, sub_obj_builder, stream, bytes_read, sub_encryption_tree);              

                sub_obj_builder.done();
            } else {
                // No need to create a sub builder as stuff is not appended anyways
                serialize<false>(msg_template, builder, stream, bytes_read, nullptr);
            }

            break;
        }
        case ros_babel_fish::MessageTypes::Bool: {
            
            if (DoSerialization) {
                uint8_t val = *reinterpret_cast<const uint8_t*>(stream + bytes_read);
                builder.appendBool(name, val != 0);
            }

            ++bytes_read;
            break;
        }
        case ros_babel_fish::MessageTypes::UInt8:{
            
            if (DoSerialization) {
                uint8_t value = *reinterpret_cast<const uint8_t*>(stream + bytes_read);
                // TODO: Maybe cast to unisgned here
                builder.append(name, value);
            }

            bytes_read += sizeof(uint8_t);
            break;
        }
        case ros_babel_fish::MessageTypes::UInt16: {
            
            if (DoSerialization) {
                uint16_t value = *reinterpret_cast<const uint16_t*>(stream + bytes_read);
                // TODO: Maybe cast to unisgned here
                builder.append(name, value);
            }

            bytes_read += sizeof(uint16_t);
            break;
        }
        case ros_babel_fish::MessageTypes::UInt32: {
            
            if (DoSerialization) {
                uint32_t value = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                // TODO: Maybe cast to unisgned here
                builder.append(name, value);
            }

            bytes_read += sizeof(uint32_t);
            break;
        }
        case ros_babel_fish::MessageTypes::UInt64: {

            if (DoSerialization) {
                uint64_t value = *reinterpret_cast<const uint64_t*>(stream + bytes_read);
                // TODO: Maybe cast to unisgned here
                builder.appendIntOrLL(name, value);
            }

            bytes_read += sizeof(uint64_t);
            break;
        }
        case ros_babel_fish::MessageTypes::Int8: {
            
            if (DoSerialization) {
                int8_t value = *reinterpret_cast<const int8_t*>(stream + bytes_read);
                builder.append(name, static_cast<int>(value));
            }

            bytes_read += sizeof(int8_t);
            break;
        }
        case ros_babel_fish::MessageTypes::Int16: {
            
            if (DoSerialization) {
                int16_t value = *reinterpret_cast<const int16_t*>(stream + bytes_read);
                builder.append(name, static_cast<int>(value));
            }

            bytes_read += sizeof(int16_t);
            break;
        }
        case ros_babel_fish::MessageTypes::Int32: {

            if (DoSerialization) {
                int32_t value = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                builder.append(name, static_cast<int>(value));
            }

            bytes_read += sizeof(int32_t);
            break;
        }
        case ros_babel_fish::MessageTypes::Int64: {
            
            if (DoSerialization) {
                int64_t value = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                builder.appendIntOrLL(name, value);
            }

            bytes_read += sizeof(int32_t);
            break;
        }
        case ros_babel_fish::MessageTypes::Float32: {
            
            if (DoSerialization) {
                float value = *reinterpret_cast<const float*>(stream + bytes_read);
                // Mongo has no specialization for float32
                builder.append(name, static_cast<double>(value));
            }

            bytes_read += sizeof(float);
            break;
        }
        case ros_babel_fish::MessageTypes::Float64: {
            
            if (DoSerialization) {
                double value = *reinterpret_cast<const double*>(stream + bytes_read);
                builder.append(name, value);
            }

            bytes_read += sizeof(double);
            break;
        }
        case ros_babel_fish::MessageTypes::String: {

            const uint8_t *begin = stream + bytes_read;
            const uint32_t length = *reinterpret_cast<const uint32_t*>(begin);
            
            if (DoSerialization) {
                builder.append(name, std::string(reinterpret_cast<const char*>(begin + 4), length));
            }

            bytes_read += length + sizeof(uint32_t);
            break;
        }
        case ros_babel_fish::MessageTypes::Time: {
            
            if (DoSerialization) {
                uint32_t secs = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                uint32_t nsecs = *reinterpret_cast<const uint32_t*>(stream + bytes_read + 4);
                // Serialize time as a nested object.
                builder.append(name, BSON("secs" << secs << "nsecs" << nsecs));
            }

            bytes_read += 8;
            break;
        }
        case ros_babel_fish::MessageTypes::Duration: {
            
            if (DoSerialization) {
                int32_t secs = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                int32_t nsecs = *reinterpret_cast<const int32_t*>(stream + bytes_read + sizeof( int32_t ));
                // Serialize duration as a nested object.
                builder.append(name, BSON("secs" << secs << "nsecs" << nsecs));
            }

            bytes_read += 8;
            break;
        }
        // TODO: Handle other cases
        case ros_babel_fish::MessageTypes::Array: {

            ssize_t length = msg_template->array.length;
            
            bool fixed_length = length >= 0;
            //stream += bytes_read;
            
            if (!fixed_length) {     
                length = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                // stream += sizeof( uint32_t );
                bytes_read += sizeof(uint32_t);
            }
              
            if (length == 0) {
                return;
            }

            switch (msg_template->array.element_type) {
                case ros_babel_fish::MessageTypes::Bool: {

                    if (DoSerialization) {

                        mongo::BSONArrayBuilder sub_array_builder = mongo::BSONArrayBuilder(builder.subarrayStart(name));

                        for (size_t idx = 0; idx < length; ++idx) {
                            sub_array_builder.appendBool(*(stream + bytes_read + idx) != 0);
                        }

                        sub_array_builder.done();

                    }

                    bytes_read += sizeof(uint8_t) * length;
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt8: {
                    
                    size_t array_byte_length = sizeof(uint8_t) * length;
                    
                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt16: {

                    size_t array_byte_length = sizeof(uint16_t) * length;

                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt32: {

                    size_t array_byte_length = sizeof(uint32_t) * length;

                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt64: {

                    size_t array_byte_length = sizeof(uint64_t) * length;

                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;
                }
                case ros_babel_fish::MessageTypes::Int8: {
                    
                    size_t array_byte_length = sizeof(int8_t) * length;

                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;;
                }
                case ros_babel_fish::MessageTypes::Int16: {

                    size_t array_byte_length = sizeof(int16_t) * length;

                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;
                }
                case ros_babel_fish::MessageTypes::Int32: {

                    size_t array_byte_length = sizeof(int32_t) * length;

                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;
                }
                case ros_babel_fish::MessageTypes::Int64: {

                    size_t array_byte_length = sizeof(int64_t) * length;

                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;
                }
                case ros_babel_fish::MessageTypes::Float32: {

                    size_t array_byte_length = sizeof(float) * length;

                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;
                }
                case ros_babel_fish::MessageTypes::Float64: {

                    size_t array_byte_length = sizeof(double) * length;

                    if (DoSerialization) {
                        builder.appendBinData(name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream + bytes_read);
                    }

                    bytes_read += array_byte_length;
                    break;
                }
                case ros_babel_fish::MessageTypes::String: {
                    
                    size_t offset = 0;

                    if (DoSerialization) {

                        mongo::BSONArrayBuilder sub_array_builder = mongo::BSONArrayBuilder(builder.subarrayStart(name));

                        for (ssize_t idx = 0; idx < length; ++idx){

                            uint32_t str_length = *reinterpret_cast<const uint32_t*>(stream + bytes_read + offset);
                            offset +=  sizeof(uint32_t);

                            sub_array_builder.append(std::string(reinterpret_cast<const char*>(stream + bytes_read + offset), str_length));
                                                        
                            offset += str_length;
                        
                        }

                        sub_array_builder.done();

                    } else {

                        size_t offset = 0;

                        for (ssize_t idx = 0; idx < length; ++idx){
                            offset += sizeof(uint32_t) + *reinterpret_cast<const uint32_t*>(stream + bytes_read + offset);
                        }

                    }

                    bytes_read += offset;
                    break;
                }
                case ros_babel_fish::MessageTypes::Time: {
                    // TODO: Does this even occur in the wild. 
                    break;
                }
                case ros_babel_fish::MessageTypes::Duration: {
                    // TODO: Does this even occur in the wild.
                    break;
                }
                case ros_babel_fish::MessageTypes::Compound: {
                        
                    if (DoSerialization) {

                        mongo::BSONArrayBuilder sub_array_builder = mongo::BSONArrayBuilder(builder.subarrayStart(name));

                        for (ssize_t idx = 0; idx < length; ++idx) {

                            OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;
                    
                            if (encryption_tree != nullptr) {

                                std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(name);

                                // No targets under this component.
                                if (matches != encryption_tree->children.end()) {
                                    sub_encryption_tree = matches->second;
                                }

                            }

                            mongo::BufBuilder& sub_obj_buff_builder = sub_array_builder.subobjStart();
                            mongo::BSONObjBuilder sub_obj_builder(sub_obj_buff_builder);

                            serialize<true>(msg_template->array.element_template, sub_obj_builder, stream, bytes_read, sub_encryption_tree);

                            sub_obj_builder.done();

                        }

                        sub_array_builder.done();

                    } else {
                        for (ssize_t idx = 0; idx < length; ++idx) {
                            serialize<false>(msg_template->array.element_template, builder, stream, bytes_read, nullptr);
                        }
                    }

                    break;
                }
                case ros_babel_fish::MessageTypes::Array:
                case ros_babel_fish::MessageTypes::None:
                    // These don't exist here
                    break;
            }

            break;
        }
        case ros_babel_fish::MessageTypes::None:
            break;
    }

}

}