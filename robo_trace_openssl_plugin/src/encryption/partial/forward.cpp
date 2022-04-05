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
// MongoCXX
#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
// Ros
#include <ros/console.h>
// Project
#include "robo_trace/processing/translation/translator.hpp"


#define STAGE_LOGGER_NAME "robo_trace_openssl_partial_encryption_forward"


namespace robo_trace::plugin::open_ssl {

PartialEncryptionForwardProcessor::PartialEncryptionForwardProcessor(const PartialEncryptionModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const robo_trace::store::Container::Ptr& metadata) 
: m_configuration(configuration), m_key_manager(key_manager) {
    
    /*
        Initialize EVP related members.
    */

    m_encryption_context = EVP_CIPHER_CTX_new();

    if (m_encryption_context == nullptr) {
        throw std::runtime_error("Could not instantiate new EVP context.");
    }

    ROS_DEBUG_STREAM_NAMED(STAGE_LOGGER_NAME, "Using encryption method: " << m_configuration->getEncryptionMethod());
    m_encryption_method = EVP_get_cipherbyname(m_configuration->getEncryptionMethod().c_str());
    
    if (m_encryption_method == nullptr) {
        throw std::runtime_error("Encryption method could not be resolved.");
    }

    /*
        Setup the IV buffer size.
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

    std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>& enrcyption_trees = m_configuration->getEncryptionTargetsTree();

    std::string message_type_adjusted = message_type;
    std::replace(message_type_adjusted.begin(), message_type_adjusted.end(), '/', '-');

    std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>::const_iterator matches = enrcyption_trees.find(message_type_adjusted);    

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

PartialEncryptionForwardProcessor::~PartialEncryptionForwardProcessor() = default;

robo_trace::processing::Mode PartialEncryptionForwardProcessor::getMode() const {
    return robo_trace::processing::Mode::CAPTURE;
}
     
void PartialEncryptionForwardProcessor::process(const robo_trace::processing::Context::Ptr& context) {

    // Sample IV for this encryption process.
    if (!RAND_bytes(&m_iv[0], m_iv.size())) {
        throw std::runtime_error("Failed sampling IV!");
    }

    context->getMetadata()->append("iv", &m_iv[0], m_iv.size());

    size_t message_stream_length = 0;
    const std::optional<const uint8_t* const> o_message_stream = context->getRosMessageStream(message_stream_length);

    if (!o_message_stream) {
        throw std::runtime_error("Unserialized message not present.");
    }

    const uint8_t* message_stream = o_message_stream.value();

    bsoncxx::builder::basic::document builder{};
  
    if (m_encryption_tree == nullptr) {
        robo_trace::processing::Translation::serialize(m_msg_template, builder, &message_stream);
    } else {
        serialize(m_msg_template, builder, &message_stream, m_encryption_tree MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA context->getStreamHandler()));
    }

    context->setBsonMessage(builder.extract());
   
}

void PartialEncryptionForwardProcessor::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, bsoncxx::builder::basic::sub_document& builder, const uint8_t** stream, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA const robo_trace::store::StreamHandler::Ptr& stream_handler)) {

    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];
        const std::string& sub_template_name = msg_template->compound.names[idx];
        
        // May need to encrypt this message.
        if (encryption_tree != nullptr &&
            // Is the current sub template a target? Yes? Encrypt!
            std::find(encryption_tree->targets.begin(), encryption_tree->targets.end(), sub_template_name) != encryption_tree->targets.end()) {

            // Currently only non authenticated methods. GCM and CCM probably not relevant anyways.
            if(!EVP_EncryptInit_ex(m_encryption_context, m_encryption_method, NULL, &m_key[0], &m_iv[0])) {
                throw std::runtime_error("Failed to initialize EVP context!");
            }

            const uint8_t* stream_encrypted_block = *stream;

            // Advance the stream without serializing it further, such that we
            // know how the size of the block to encrypt.
            robo_trace::processing::Translation::advance(sub_template, &stream_encrypted_block);

            const int encrypted_block_length = (int) (stream_encrypted_block - *stream);
            // Possible padding
            const int block_size = EVP_CIPHER_CTX_block_size(m_encryption_context);
            // The ciper text may be longer than the original text due to possible padding.
            const int cipher_max_legth = encrypted_block_length + block_size;

            /* 
                No way arround possibly allocating a new chunk on the heap here, as we
                can't access the byte buffer of the Bson builder.
            */
            m_cipher_buffer.resize(cipher_max_legth);
            int cipher_text_length = 0;

            if(!EVP_EncryptUpdate(m_encryption_context, &m_cipher_buffer[0], &cipher_text_length, *stream, encrypted_block_length)) {
                throw std::runtime_error("Failed feeding message for encryption.");
            }

            int length = 0;
            // Might need to add some padding.
            if(!EVP_EncryptFinal_ex(m_encryption_context, &m_cipher_buffer[0] + cipher_text_length, &length)) {
                throw std::runtime_error("Failed finalizing encryption.");
            }

#ifdef MODULE_PARTIAL_ENCRYPTION_OFFLOAD_ENCRYPTED_BLOBS
            const bsoncxx::types::bson_value::value id = stream_handler->store("TODO", &m_cipher_buffer[0], cipher_text_length + length);  
            builder.append(bsoncxx::builder::basic::kvp(sub_template_name, id));
#else
            bsoncxx::types::b_binary wrapper;
            wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
            wrapper.size = cipher_text_length + length;
            wrapper.bytes = &m_cipher_buffer[0];

            builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));
#endif

            *stream += encrypted_block_length;
                
        } else {
            serialize(sub_template, sub_template_name, builder, stream, encryption_tree MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA stream_handler));
        }
    }

}

void PartialEncryptionForwardProcessor::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const std::string& name, bsoncxx::builder::basic::sub_document& builder, const uint8_t** stream, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA const robo_trace::store::StreamHandler::Ptr& stream_handler)) {

    switch (msg_template->type) {
        case ros_babel_fish::MessageTypes::Compound: {
          
            PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;
            
            if (encryption_tree != nullptr) {

                std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(name);

                // No targets under this component.
                if (matches != encryption_tree->children.end()) {
                    sub_encryption_tree = matches->second;
                }

            }

            if (sub_encryption_tree == nullptr) {
                // Nothing will be encrypted from this point on, so we can use the default method
                // without all the checks and stuff. 
                robo_trace::processing::Translation::serialize(msg_template, builder, stream);
            } else {
                builder.append(bsoncxx::builder::basic::kvp(name, 
                    [this, &msg_template, &sub_encryption_tree, stream MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA &stream_handler)](bsoncxx::builder::basic::sub_document sub_document_builder) mutable {
                        this->serialize(msg_template, sub_document_builder, stream, sub_encryption_tree MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA stream_handler));
                    }
                ));
            }

            break;
        }
        case ros_babel_fish::MessageTypes::Bool: {
            
            const uint8_t val = *reinterpret_cast<const uint8_t*>(*stream);
            *stream += sizeof(uint8_t);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_bool{val != 0}));

            break;
        }
        case ros_babel_fish::MessageTypes::UInt8:{
            
            const uint8_t value = *reinterpret_cast<const uint8_t*>(*stream);
            *stream += sizeof(uint8_t);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));

            break;
        }
        case ros_babel_fish::MessageTypes::UInt16: {
            
            const uint16_t value = *reinterpret_cast<const uint16_t*>(*stream);
            *stream += sizeof(uint16_t);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
        
            break;
        }
        case ros_babel_fish::MessageTypes::UInt32: {
            
            const uint32_t value = *reinterpret_cast<const uint32_t*>(*stream);
            *stream += sizeof(uint32_t);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            
            break;
        }
        case ros_babel_fish::MessageTypes::UInt64: {

            const uint64_t value = *reinterpret_cast<const uint64_t*>(*stream);
            *stream += sizeof(uint64_t);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int64{static_cast<int64_t>(value)}));

            break;
        }
        case ros_babel_fish::MessageTypes::Int8: {
            
            const int8_t value = *reinterpret_cast<const int8_t*>(*stream);
            *stream += sizeof(int8_t);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            
            break;
        }
        case ros_babel_fish::MessageTypes::Int16: {
            
            const int16_t value = *reinterpret_cast<const int16_t*>(*stream);
            *stream += sizeof(int16_t);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            
            break;
        }
        case ros_babel_fish::MessageTypes::Int32: {

            const int32_t value = *reinterpret_cast<const int32_t*>(*stream);
            *stream += sizeof(int32_t);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            
            break;
        }
        case ros_babel_fish::MessageTypes::Int64: {
            
            const int64_t value = *reinterpret_cast<const int32_t*>(*stream);
            *stream += sizeof(int64_t);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int64{static_cast<int64_t>(value)}));
            
            break;
        }
        case ros_babel_fish::MessageTypes::Float32: {
            
            const float value = *reinterpret_cast<const float*>(*stream);
            *stream += sizeof(float);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_double{static_cast<double>(value)}));
            
            break;
        }
        case ros_babel_fish::MessageTypes::Float64: {
            
            const double value = *reinterpret_cast<const double*>(*stream);
            *stream += sizeof(double);

            builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_double{value}));
            
            break;
        }
        case ros_babel_fish::MessageTypes::String: {

            const uint32_t length = *reinterpret_cast<const uint32_t*>(*stream);
            *stream += sizeof(uint32_t);

            builder.append(bsoncxx::builder::basic::kvp(name, std::string(reinterpret_cast<const char*>(*stream), length)));
            *stream += length * sizeof(char);

            break;
        }
        case ros_babel_fish::MessageTypes::Time: {
            
            builder.append(bsoncxx::builder::basic::kvp(name, 
                [stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
                    
                    // TODO: Could convert to int32 right away... The compiler will optimize it for us (fingers crossed).
                    const uint32_t secs = *reinterpret_cast<const uint32_t*>(*stream);
                    *stream += sizeof(uint32_t);
                    
                    sub_document_builder.append(bsoncxx::builder::basic::kvp("secs", bsoncxx::types::b_int32{static_cast<int32_t>(secs)}));
                    
                    const uint32_t nsecs = *reinterpret_cast<const uint32_t*>(*stream);   
                    *stream += sizeof(uint32_t);
                    
                    sub_document_builder.append(bsoncxx::builder::basic::kvp("nsecs", bsoncxx::types::b_int32{static_cast<int32_t>(nsecs)}));  

            }));
            
            break;
        }
        case ros_babel_fish::MessageTypes::Duration: {
            
            builder.append(bsoncxx::builder::basic::kvp(name, 
                [stream](bsoncxx::builder::basic::sub_document sub_document_builder) {

                    const int32_t secs = *reinterpret_cast<const int32_t*>(*stream);
                    *stream += sizeof(int32_t);

                    sub_document_builder.append(bsoncxx::builder::basic::kvp("secs", bsoncxx::types::b_int32{secs}));
                    
                    const int32_t nsecs = *reinterpret_cast<const int32_t*>(*stream);   
                    *stream += sizeof(int32_t);
                    
                    sub_document_builder.append(bsoncxx::builder::basic::kvp("nsecs", bsoncxx::types::b_int32{nsecs}));
                    
            }));
            

            break;
        }
        // TODO: Handle other cases
        case ros_babel_fish::MessageTypes::Array: {

            ssize_t length = msg_template->array.length;  
            const bool fixed_length = length >= 0;
            
            if (!fixed_length) {     
                length = *reinterpret_cast<const uint32_t*>(*stream);
                *stream += sizeof(uint32_t);
            }
              
            if (length == 0) {
                return;
            }

            switch (msg_template->array.element_type) {
                case ros_babel_fish::MessageTypes::Bool: {

                    builder.append(bsoncxx::builder::basic::kvp(name,
                        [stream, length](bsoncxx::builder::basic::sub_array array_builder) {  
                            for (size_t idx = 0; idx < length; ++idx) {

                                array_builder.append(bsoncxx::types::b_bool{*stream != 0});
                                *stream += sizeof(uint8_t);

                            }
                    })); 
                   
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt8: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(uint8_t) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt16: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(uint16_t) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt32: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(uint32_t) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt64: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(uint64_t) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Int8: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(int8_t) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Int16: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(int16_t) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Int32: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(int32_t) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Int64: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(int64_t) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Float32: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(float) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Float64: {
                    robo_trace::processing::Translation::upload(builder, name, stream, sizeof(double) * length);
                    break;
                }
                case ros_babel_fish::MessageTypes::String: {
                    
                    builder.append(bsoncxx::builder::basic::kvp(name, 
                        [stream, length](bsoncxx::builder::basic::sub_array array_builder) {
                            for (size_t idx = 0; idx < length; ++idx){

                                const uint32_t str_length = *reinterpret_cast<const uint32_t*>(*stream);
                                *stream += sizeof(uint32_t);
                                
                                array_builder.append(std::string(reinterpret_cast<const char*>(*stream), str_length));
                                *stream += str_length * sizeof(char);

                            }       
                    })); 

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
                     
                    builder.append(bsoncxx::builder::basic::kvp(name, 
                        [this, &msg_template, &encryption_tree, &name, stream, length MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA &stream_handler)](bsoncxx::builder::basic::sub_array array_builder) {
                            
                            PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;
                
                            if (encryption_tree != nullptr) {

                                std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(name);

                                // No targets under this component.
                                if (matches != encryption_tree->children.end()) {
                                    sub_encryption_tree = matches->second;
                                }

                            }
                            
                            for (size_t idx = 0; idx < length; ++idx) {
                                if (sub_encryption_tree == nullptr) {
                                    array_builder.append([this, &msg_template, &sub_encryption_tree, stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
                                        robo_trace::processing::Translation::serialize(msg_template->array.element_template, sub_document_builder, stream);
                                    });    
                                } else {
                                    array_builder.append([this, &msg_template, &sub_encryption_tree, stream MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA &stream_handler)](bsoncxx::builder::basic::sub_document sub_document_builder) {
                                        this->serialize(msg_template->array.element_template, sub_document_builder, stream, sub_encryption_tree MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA stream_handler));
                                    });
                                }               
                            }
                    }));

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