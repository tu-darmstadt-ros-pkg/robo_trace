import QtQuick 2.6
import QtQuick.Layouts 1.0
import QtQuick.Controls.Material 2.2
import QtQuick.Controls 2.2
import QtQuick.Dialogs 1.1
import QtMultimedia 5.4
import Ros 1.0


Item {
    id: page
    anchors.fill: parent
    focus: true

	// == Configuration ==

	readonly property string adaptor_node_name: "robo_trace_ui_adaptor"

    readonly property string topic_name_playback_meta: "meta"
    readonly property string topic_name_playback_progression: "progression"
	readonly property string topic_name_display_error: "error"

	readonly property string topic_name_signal_ui_init_completed: "signal_ui_initialized"
	readonly property string topic_name_signal_rt_init_completed: "signal_rt_initialized"
	readonly property string topic_name_signal_error_accepted: "signal_error_accepted"
	readonly property string topic_name_signal_open_configure: "signal_open_configure"

    readonly property string service_name_set_playback_state: "set_playback_state"
	readonly property string service_message_type_set_playback_state: "robo_trace_ui_adaptor/SetPlaybackState"
    
	readonly property string service_name_set_playback_time: "set_playback_time"
	readonly property string service_message_type_set_playback_time: "robo_trace_ui_adaptor/SetPlaybackTime"
	
	readonly property string service_name_set_playback_configuration: "set_playback_configuration"
	readonly property string service_message_type_set_playback_configuration: "robo_trace_ui_adaptor/SetPlaybackConfiguration"

	// == Constants ==

	readonly property string topic_path_playback_meta: "/" + adaptor_node_name + "/" + topic_name_playback_meta
	readonly property string topic_path_playback_progression: "/" + adaptor_node_name + "/" + topic_name_playback_progression
	readonly property string topic_path_display_error: "/" + adaptor_node_name + "/" + topic_name_display_error

	readonly property string topic_path_signal_ui_init_completed: "/" + adaptor_node_name + "/" + topic_name_signal_ui_init_completed
	readonly property string topic_path_signal_rt_init_completed: "/" + adaptor_node_name + "/" + topic_name_signal_rt_init_completed
	readonly property string topic_path_signal_error_accepted:  "/" + adaptor_node_name + "/" + topic_name_signal_error_accepted
	readonly property string topic_path_signal_open_configure:  "/" + adaptor_node_name + "/" + topic_name_signal_open_configure

	readonly property string service_path_set_playback_state: "/" + adaptor_node_name + "/" + service_name_set_playback_state
	readonly property string service_path_set_playback_time: "/" + adaptor_node_name + "/" + service_name_set_playback_time
	readonly property string service_path_set_playback_configure: "/" + adaptor_node_name + "/" + service_name_set_playback_configuration
	
	
	// == Publisher ==

	property var publisher_signal_ui_init_completed: Ros.advertise("std_msgs/Empty", topic_path_signal_ui_init_completed, 1, false)
	property var publisher_signal_error_accepted: Ros.advertise("std_msgs/Empty", topic_path_signal_error_accepted, 1, false)

	// == Init ==

	Component.onCompleted: {
		Ros.init("robo_trace_ui")
	}

	// == Subscriber ==

	Subscriber {
		id: subscriber_playback_meta
		topic: topic_path_playback_meta
		onNewMessage: onPlaybackMetaUpdate(message)
	}

	Subscriber {
		id: subscriber_rt_init_completed
		topic: topic_path_signal_rt_init_completed
		onNewMessage: onRtAdaptorInitialized(message)
	}

	Subscriber {
		id: subscriber_playback_progression
		topic: topic_path_playback_progression
		onNewMessage: onPlaybackProgressionUpdate(message)
	}

	Subscriber {
		id: subscriber_error_display
		topic: topic_path_display_error
		onNewMessage: onErrorMessage(message)
	}

	Subscriber {
		id: subscriber_open_configure_display
		topic: topic_path_signal_open_configure
		onNewMessage: onPlaybackConfigure(message)
	} 
	
	// == Functions ==
	
	function onRtAdaptorInitialized(message) {
		console.log("Adaptor is initialized. ACKing.")
		// TODO: Somehow the message needs to be send twice here for in
		// order to get some signal?!
		publisher_signal_ui_init_completed.publish({})
		publisher_signal_ui_init_completed.publish({})
	}

	function onPlaybackMetaUpdate(message) {
		console.log("Playback max time: " + message.length_formated)
		playback_time_text_max.text = message.length_formated
	}

	function onPlaybackProgressionUpdate(message) {
		console.log("Playback progressed to: "+ message.formated + " (share " + message.progression + ")")
		playback_time_text_current.text = message.formated
		playback_time_slider.value = message.progression
	}

	function onErrorMessage(message) {
		console.log("Error: " + message.data)
		error_popup.text = message.data
		error_popup.open()
	}	

	function onPlaybackConfigure(message) {
		console.log("Received reconfiguration request.")
		loading_dialog_playback_input.text = ""
		loading_dialog_from_input.text = ""
		loading_dialog_to_input.text = ""
		loading_dialog_error_text.visible = false
		loading_dialog_error_text.text = "Error."
		dialog_define_replay.open()
	}

	function onStatusIndication(message) {

		if (message.success) {
			return
		}

		error_popup.text = message.data
		error_popup.open()

	}

	// == Popups ==

	MessageDialog {
		id: error_popup
		modality: Qt.ApplicationModal
		icon: StandardIcon.Critical
		standardButtons:  StandardButton.Ok
		title: "Error!"
		text: "Something went wrong..."

		onAccepted: {
			publisher_signal_error_accepted.publish({})
		}

	}

	Dialog {
        id: dialog_define_replay
		modal: true   
		closePolicy: Popup.NoAutoClose
		focus: true  
        width: 300
		x: (parent.width - width) / 2
    	y: (parent.height - height) / 2
		title: "Load Recording."
        standardButtons: StandardButton.Ok

		onAccepted: {

			loading_dialog_error_text.text = "Loading ..."
			loading_dialog_error_text.visible = true

			dialog_define_replay.open()
			dialog_define_replay.standardButtons = Dialog.NoButton

			var result = Service.callAsync(service_path_set_playback_configure, service_message_type_set_playback_configuration, {
												name: loading_dialog_playback_input.text,
												to: loading_dialog_from_input.text,
												from: loading_dialog_to_input.text
											}, function (result) {
												console.log("Done loading: " + result.status.message)

												dialog_define_replay.standardButtons = StandardButton.Ok

												if (result.status.success) {
													dialog_define_replay.close()
												} else {
													loading_dialog_error_text.text = result.status.message
													loading_dialog_error_text.visible = true
												}

											})

		}

		Column {
			id: loading_dialog_container
			width: parent.width
			anchors.fill: parent
			anchors.leftMargin: 10

			GridLayout {	
				id: loading_dialog_form
				width: loading_dialog_container.width
				columns: 2
				rows: 3
				
				Text {
					id: loading_dialog_playback_name
					text: "Name: "
					Layout.alignment: Qt.AlignRight
				}

				TextInput {
					id: loading_dialog_playback_input
					Layout.fillWidth: true
					focus: true
					readOnly: false
					// width: loading_dialog_container.width - loading_dialog_playback_name.width
					cursorVisible: true
					cursorPosition: 0
					
					onVisibleChanged: if(visible) loading_dialog_playback_input.forceActiveFocus()

					Rectangle {
						color: "grey"
						z : -1
						anchors {

							left: parent.left
							right: parent.right
							top: parent.top
							bottom: parent.bottom

							topMargin    : loading_dialog_playback_name.height + 1
							bottomMargin : -2
							leftMargin   : 0
							rightMargin  : 0
			
						}
					}
				}
					
				Text {
					id: loading_dialog_from_name
					text: "From: "
					Layout.alignment: Qt.AlignRight
				}

				TextInput {
					id: loading_dialog_from_input
					Layout.fillWidth: true
					readOnly: false

					Rectangle {
						color: "grey"
						z : -1
						anchors {

							left: parent.left
							right: parent.right
							top: parent.top
							bottom: parent.bottom

							topMargin    : loading_dialog_from_name.height + 1
							bottomMargin : -2
							leftMargin   : 0
							rightMargin  : 0
			
						}
					}
				}
				
				Text {
					id: loading_dialog_to_name
					text: "To: "
					Layout.alignment: Qt.AlignRight
				}

				TextInput {
					id: loading_dialog_to_input
					Layout.fillWidth: true
					readOnly: false

					Rectangle {
						color: "grey"
						z : -1
						anchors {

							left: parent.left
							right: parent.right
							top: parent.top
							bottom: parent.bottom

							topMargin : loading_dialog_to_name.height + 1
							bottomMargin : -2
							leftMargin   : 0
							rightMargin  : 0
			
						}
					}
				}
			
			}

			Text {
				id: loading_dialog_error_text
				topPadding: 20
				text: "Error."
				visible: false
			}

		}
    }

	// == Main Ui ==

	RowLayout {
		anchors.margins: 12
		anchors.top: page.top
		anchors.left: page.left
		anchors.right: page.right
		height: 20

		Text{
			id: playback_time_text_current	
			font.bold: true
			text: "00:00"
			color: "white"
		}

	
		Slider {
			id: playback_time_slider
			Layout.fillWidth: true
			from: 0
			to: 1
			value: 0
			orientation: Qt.Horizontal
			snapMode: Slider.NoSnap
			
			onMoved: {

				var result = Service.callAsync(service_path_set_playback_time, service_message_type_set_playback_time, {
										progression: playback_time_slider.value
									})

				onStatusIndication(result.status)

			}

		}

		Text{
			id: playback_time_text_max
			font.bold: true
			text: "00:00"
			color: "white"
		}

		Item {
			width: 10
		}
	
		Button {
			text: "Play"
			onClicked: {

				var result = Service.callAsync(service_path_set_playback_state, service_message_type_set_playback_state, {
										run: true
									})

				onStatusIndication(result.status)

			}
		}
		
		Button {
			text: "Pause"
			onClicked: {

				var result = Service.callAsync(service_path_set_playback_state, service_message_type_set_playback_state, {
										run: false
									})

				onStatusIndication(result.status)

			}
		}
					
	
	} // Row layout

} // Item