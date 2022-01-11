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

#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
#define GET_READ_POSITION(stream, bytes_read) stream + bytes_read
#else
#define GET_READ_POSITION(stream, bytes_read) *stream
#endif

#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
#define ADVANCE_READ_POSITION(stream, bytes_read, read) bytes_read += read
#else
#define ADVANCE_READ_POSITION(stream, bytes_read, read) *stream += read
#endif

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
    const std::optional<const uint8_t* const> o_message_stream = context->getUnserializedMessage(message_stream_length);

    if (!o_message_stream) {
        throw std::runtime_error("Unserialized message not present.");
    }

    const uint8_t* message_stream = o_message_stream.value();

    bsoncxx::builder::basic::document builder{};
    size_t bytes_read = 0;

    // Starting with DoDeserialize = true
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
    serialize<true>(m_msg_template, builder, message_stream, bytes_read, m_encryption_tree);
#else
    serialize<true>(m_msg_template, builder, &message_stream, m_encryption_tree);
#endif

    context->setSerializedMessage(builder.extract());
   
}

#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
template<bool DoSerialization>
void PartialEncryptionForwardProcessor::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, bsoncxx::builder::basic::sub_document& builder, const uint8_t* stream, size_t& bytes_read, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree) {
#else
template<bool DoSerialization>
void PartialEncryptionForwardProcessor::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, bsoncxx::builder::basic::sub_document& builder, const uint8_t** stream, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree) {
#endif

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

#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
            // We need to know how many byte to encrypt and also advance the read bytes index!
            size_t bytes_read_encrypted_block = bytes_read;
            
            serialize<false>(sub_template, sub_template_name, builder, stream, bytes_read_encrypted_block, nullptr);

            // How many bytes are there to be encrypted?
            const int encrypted_block_length = (int) bytes_read_encrypted_block - bytes_read;
#else
            const uint8_t* stream_encrypted_block = *stream;
            serialize<false>(sub_template, sub_template_name, builder, &stream_encrypted_block, nullptr);
            const int encrypted_block_length = (int) (stream_encrypted_block - *stream);
#endif

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

            if(!EVP_EncryptUpdate(m_encryption_context, &m_cipher_buffer[0], &cipher_text_length, GET_READ_POSITION(stream, bytes_read), encrypted_block_length)) {
                throw std::runtime_error("Failed feeding message for encryption.");
            }

            int length = 0;
            // Usually does not add anything to the cipher text, but might i.e. for padding.
            if(!EVP_EncryptFinal_ex(m_encryption_context, &m_cipher_buffer[0] + cipher_text_length, &length)) {
                throw std::runtime_error("Failed finalizing encryption.");
            }

            bsoncxx::types::b_binary wrapper;
            wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
            wrapper.size = cipher_text_length + length;
            wrapper.bytes = &m_cipher_buffer[0];

            builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));

            // Arrrg.
            // builder.appendBinData(sub_template_name, cipher_text_length + length, mongo::BinDataType::BinDataGeneral, &m_cipher_buffer[0]);

            ADVANCE_READ_POSITION(stream, bytes_read, encrypted_block_length);
            //bytes_read = bytes_read_encrypted_block;
                
        } else {
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
            serialize<DoSerialization>(sub_template, sub_template_name, builder, stream, bytes_read, encryption_tree);
#else
            serialize<DoSerialization>(sub_template, sub_template_name, builder, stream, encryption_tree);
#endif
        }
    }

}

#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
template<bool DoSerialization>
void PartialEncryptionForwardProcessor::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const std::string& name, bsoncxx::builder::basic::sub_document& builder, const uint8_t* stream, size_t& bytes_read, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree) {
#else
template<bool DoSerialization>
void PartialEncryptionForwardProcessor::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const std::string& name, bsoncxx::builder::basic::sub_document& builder, const uint8_t** stream, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree) {
#endif

    switch (msg_template->type) {
        case ros_babel_fish::MessageTypes::Compound: {
          
            if (DoSerialization) {     
                
                PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;
                
                if (encryption_tree != nullptr) {

                    std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(name);

                    // No targets under this component.
                    if (matches != encryption_tree->children.end()) {
                        sub_encryption_tree = matches->second;
                    }

                }

                builder.append(bsoncxx::builder::basic::kvp(name, 
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
                    [this, &msg_template, &bytes_read, &sub_encryption_tree, stream](bsoncxx::builder::basic::sub_document sub_document_builder) mutable {
                        this->serialize<true>(msg_template, sub_document_builder, stream, bytes_read, sub_encryption_tree);
#else
                    [this, &msg_template, &sub_encryption_tree, stream](bsoncxx::builder::basic::sub_document sub_document_builder) mutable {
                        this->serialize<true>(msg_template, sub_document_builder, stream, sub_encryption_tree);
#endif 
                }));

            /*
                mongo::BufBuilder& sub_buff_builder = builder.subobjStart(name);
                mongo::BSONObjBuilder sub_obj_builder(sub_buff_builder);

                serialize<true>(msg_template, sub_obj_builder, stream, bytes_read, sub_encryption_tree);              

                sub_obj_builder.done();
            */
            } else {
                // No need to create a sub builder as stuff is not appended anyways
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
                serialize<false>(msg_template, builder, stream, bytes_read, nullptr);
#else
                serialize<false>(msg_template, builder, stream, nullptr);
#endif
            }

            break;
        }
        case ros_babel_fish::MessageTypes::Bool: {
            
            if (DoSerialization) {
                const uint8_t val = *reinterpret_cast<const uint8_t*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_bool{val != 0}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(uint8_t));
            break;
        }
        case ros_babel_fish::MessageTypes::UInt8:{
            
            if (DoSerialization) {
                const uint8_t value = *reinterpret_cast<const uint8_t*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(uint8_t));
            break;
        }
        case ros_babel_fish::MessageTypes::UInt16: {
            
            if (DoSerialization) {
                const uint16_t value = *reinterpret_cast<const uint16_t*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(uint16_t));
            break;
        }
        case ros_babel_fish::MessageTypes::UInt32: {
            
            if (DoSerialization) {
                const uint32_t value = *reinterpret_cast<const uint32_t*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(uint32_t));
            break;
        }
        case ros_babel_fish::MessageTypes::UInt64: {

            if (DoSerialization) {
                const uint64_t value = *reinterpret_cast<const uint64_t*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int64{static_cast<int64_t>(value)}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(uint64_t));
            break;
        }
        case ros_babel_fish::MessageTypes::Int8: {
            
            if (DoSerialization) {
                const int8_t value = *reinterpret_cast<const int8_t*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(int8_t));
            break;
        }
        case ros_babel_fish::MessageTypes::Int16: {
            
            if (DoSerialization) {
                const int16_t value = *reinterpret_cast<const int16_t*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(int16_t));
            break;
        }
        case ros_babel_fish::MessageTypes::Int32: {

            if (DoSerialization) {
                const int32_t value = *reinterpret_cast<const int32_t*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(int32_t));
            break;
        }
        case ros_babel_fish::MessageTypes::Int64: {
            
            if (DoSerialization) {
                const int64_t value = *reinterpret_cast<const int32_t*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_int64{static_cast<int64_t>(value)}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(int64_t));
            break;
        }
        case ros_babel_fish::MessageTypes::Float32: {
            
            if (DoSerialization) {
                const float value = *reinterpret_cast<const float*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_double{static_cast<double>(value)}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(float));
            break;
        }
        case ros_babel_fish::MessageTypes::Float64: {
            
            if (DoSerialization) {
                const double value = *reinterpret_cast<const double*>(GET_READ_POSITION(stream, bytes_read));
                builder.append(bsoncxx::builder::basic::kvp(name, bsoncxx::types::b_double{value}));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(double));
            break;
        }
        case ros_babel_fish::MessageTypes::String: {

            const uint32_t length = *reinterpret_cast<const uint32_t*>(GET_READ_POSITION(stream, bytes_read));
            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(uint32_t));

            if (DoSerialization) {
                builder.append(bsoncxx::builder::basic::kvp(name, std::string(reinterpret_cast<const char*>(GET_READ_POSITION(stream, bytes_read)), length)));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, length * sizeof(char));
            break;
        }
        case ros_babel_fish::MessageTypes::Time: {
            
            if (DoSerialization) {
                builder.append(bsoncxx::builder::basic::kvp(name, 
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
                            [stream, bytes_read](bsoncxx::builder::basic::sub_document sub_document_builder) {
#else
                            [stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
#endif

                        // TODO: Could convert to int32 right away...
                        const uint32_t secs = *reinterpret_cast<const uint32_t*>(GET_READ_POSITION(stream, bytes_read));
                        sub_document_builder.append(bsoncxx::builder::basic::kvp("secs", bsoncxx::types::b_int32{static_cast<int32_t>(secs)}));
                        
                        const uint32_t nsecs = *reinterpret_cast<const uint32_t*>(GET_READ_POSITION(stream, bytes_read) + sizeof(uint32_t));   
                        sub_document_builder.append(bsoncxx::builder::basic::kvp("nsecs", bsoncxx::types::b_int32{static_cast<int32_t>(nsecs)}));
                        
                }));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, 2 * sizeof(uint32_t));
            break;
        }
        case ros_babel_fish::MessageTypes::Duration: {
            
            if (DoSerialization) {
                builder.append(bsoncxx::builder::basic::kvp(name, 
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
                            [stream, bytes_read](bsoncxx::builder::basic::sub_document sub_document_builder) {
#else
                            [stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
#endif

                        int32_t secs = *reinterpret_cast<const int32_t*>(GET_READ_POSITION(stream, bytes_read));
                        sub_document_builder.append(bsoncxx::builder::basic::kvp("secs", bsoncxx::types::b_int32{secs}));
                        
                        int32_t nsecs = *reinterpret_cast<const int32_t*>(GET_READ_POSITION(stream, bytes_read) + sizeof(int32_t));   
                        sub_document_builder.append(bsoncxx::builder::basic::kvp("nsecs", bsoncxx::types::b_int32{nsecs}));
                        
                }));
            }

            ADVANCE_READ_POSITION(stream, bytes_read, 2 * sizeof(int32_t));
            break;
        }
        // TODO: Handle other cases
        case ros_babel_fish::MessageTypes::Array: {

            ssize_t length = msg_template->array.length;  
            const bool fixed_length = length >= 0;
            
            if (!fixed_length) {     
                length = *reinterpret_cast<const uint32_t*>(GET_READ_POSITION(stream, bytes_read));
                ADVANCE_READ_POSITION(stream, bytes_read, sizeof(uint32_t));
            }
              
            if (length == 0) {
                return;
            }

            switch (msg_template->array.element_type) {
                case ros_babel_fish::MessageTypes::Bool: {

                    if (DoSerialization) {
                        builder.append(bsoncxx::builder::basic::kvp(name,
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
                            [stream, bytes_read, length](bsoncxx::builder::basic::sub_array array_builder) {
#else
                            [stream, length](bsoncxx::builder::basic::sub_array array_builder) {
#endif
                                const uint8_t* begin = GET_READ_POSITION(stream, bytes_read);

                                for (size_t idx = 0; idx < length; ++idx) {
                                    array_builder.append(bsoncxx::types::b_bool{*(begin + idx) != 0});
                                }

                        })); 
                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, length * sizeof(uint8_t));
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt8: {
                    
                    size_t array_byte_length = sizeof(uint8_t) * length;
                    
                    if (DoSerialization) {
                        
                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));

                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt16: {

                    const size_t array_byte_length = sizeof(uint16_t) * length;

                    if (DoSerialization) {
                        
                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));
                        
                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt32: {

                    size_t array_byte_length = sizeof(uint32_t) * length;

                    if (DoSerialization) {

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));

                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt64: {

                    size_t array_byte_length = sizeof(uint64_t) * length;

                    if (DoSerialization) {
                        
                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));

                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Int8: {
                    
                    size_t array_byte_length = sizeof(int8_t) * length;

                    if (DoSerialization) {
                        
                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));

                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;;
                }
                case ros_babel_fish::MessageTypes::Int16: {

                    size_t array_byte_length = sizeof(int16_t) * length;

                    if (DoSerialization) {
                        
                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));

                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Int32: {

                    size_t array_byte_length = sizeof(int32_t) * length;

                    if (DoSerialization) {
                        
                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));

                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Int64: {

                    size_t array_byte_length = sizeof(int64_t) * length;

                    if (DoSerialization) {
                        
                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));

                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Float32: {

                    size_t array_byte_length = sizeof(float) * length;

                    if (DoSerialization) {
                        
                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));

                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;
                }
                case ros_babel_fish::MessageTypes::Float64: {

                    size_t array_byte_length = sizeof(double) * length;

                    if (DoSerialization) {
                        
                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = GET_READ_POSITION(stream, bytes_read);

                        builder.append(bsoncxx::builder::basic::kvp(name, wrapper));

                    }

                    ADVANCE_READ_POSITION(stream, bytes_read, array_byte_length);
                    break;
                }
                case ros_babel_fish::MessageTypes::String: {
                    
                    size_t offset = 0;

                    if (DoSerialization) {

                        builder.append(bsoncxx::builder::basic::kvp(name, 
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
                            [&bytes_read, stream, length](bsoncxx::builder::basic::sub_array array_builder) {
#else
                            [stream, length](bsoncxx::builder::basic::sub_array array_builder) {
#endif
                                for (size_t idx = 0; idx < length; ++idx){

                                    const uint32_t str_length = *reinterpret_cast<const uint32_t*>(GET_READ_POSITION(stream, bytes_read));
                                    ADVANCE_READ_POSITION(stream, bytes_read, sizeof(uint32_t));
                                  
                                    array_builder.append(std::string(reinterpret_cast<const char*>(GET_READ_POSITION(stream, bytes_read)), str_length));
                                    ADVANCE_READ_POSITION(stream, bytes_read, str_length * sizeof(char));

                                }
                                
                        })); 

                    } else {
                        
                        //const uint8_t* begin = GET_READ_POSITION(stream, bytes_read);
                        //size_t offset = 0;

                        for (ssize_t idx = 0; idx < length; ++idx){
                            ADVANCE_READ_POSITION(stream, bytes_read, sizeof(uint32_t) + *reinterpret_cast<const uint32_t*>(GET_READ_POSITION(stream, bytes_read)) * sizeof(char));
                            //offset += sizeof(uint32_t) + *reinterpret_cast<const uint32_t*>(begin + offset);
                        }

                    }

                    //ADVANCE_READ_POSITION(stream, bytes_read, offset);
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
                        
                        builder.append(bsoncxx::builder::basic::kvp(name, 
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
                            [this, &msg_template, &encryption_tree, &bytes_read, &name, stream, length](bsoncxx::builder::basic::sub_array array_builder) {
#else
                            [this, &msg_template, &encryption_tree, &name, stream, length](bsoncxx::builder::basic::sub_array array_builder) {
#endif
                                for (size_t idx = 0; idx < length; ++idx) {

                                    PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;
                    
                                    if (encryption_tree != nullptr) {

                                        std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(name);

                                        // No targets under this component.
                                        if (matches != encryption_tree->children.end()) {
                                            sub_encryption_tree = matches->second;
                                        }

                                    }

#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
                                    array_builder.append([this, &msg_template, &sub_encryption_tree, &bytes_read, stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
                                        this->serialize<true>(msg_template, sub_document_builder, stream, bytes_read, sub_encryption_tree);
                                    });
#else
                                    array_builder.append([this, &msg_template, &sub_encryption_tree, stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
                                        this->serialize<true>(msg_template, sub_document_builder, stream, sub_encryption_tree);
                                    });
#endif
                                        
                                }
                        }));

                    } else {
                        for (ssize_t idx = 0; idx < length; ++idx) {
#ifdef MODULE_PARTIAL_ENCRYPTION_FORWARD_VALIDATE_BYTES
                            serialize<false>(msg_template->array.element_template, builder, stream, bytes_read, nullptr);
#else
                            serialize<false>(msg_template->array.element_template, builder, stream, nullptr);
#endif
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