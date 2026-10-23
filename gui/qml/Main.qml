pragma ComponentBehavior: Bound
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

ApplicationWindow {
    id: root
    required property QtObject appController
    QtObject {
        id: fallbackController
        property string profileJson: "{}"
        property var profileNames: []
        property var launchArgumentNames: []
        property bool runActive: false
        property string state: "Idle"
        property string message: ""
        function requestQuit() {}
        function stopMotion() {}
        function runPreflight() {}
        function startRun() {}
        function requestHome() {}
        function enableTracking() {}
        function stopAll() {}
        function forceStopAfterEstop() {}
        function refreshRecordings() {}
        function syncNow() {}
        function selectProfile(name) {}
        function saveProfile(name) {}
        function setOption(name, value) {}
        function setExpertOption(name, value) {}
        function openPath(path) {}
    }
    property QtObject controller: appController || fallbackController
    width: 1440
    height: 900
    minimumWidth: 1120
    minimumHeight: 720
    visible: true
    title: "DracoViLoc Control Center"
    color: "#0B1118"
    property int currentPage: 0
    property int cameraRevision: 0
    property bool shutdownApproved: false
    property var profile: JSON.parse(root.controller.profileJson)
    property var liveStatus: ({})

    palette.window: "#0B1118"
    palette.windowText: "#E8F0F7"
    palette.base: "#111A24"
    palette.text: "#E8F0F7"
    palette.button: "#182431"
    palette.buttonText: "#E8F0F7"
    palette.highlight: "#35C6D0"
    palette.highlightedText: "#071014"

    onClosing: function(close) {
        if (root.shutdownApproved) {
            close.accepted = true
            return
        }
        close.accepted = false
        if (root.controller.runActive)
            closeDialog.open()
        else
            root.controller.requestQuit()
    }

    Connections {
        target: root.controller
        function onProfileChanged() { root.profile = JSON.parse(root.controller.profileJson) }
        function onStatusChanged(payload) { root.liveStatus = JSON.parse(payload) }
        function onPreflightFinished(payload, ok) {
            preflightModel.clear()
            let rows = JSON.parse(payload)
            for (let row of rows) preflightModel.append(row)
            preflightDrawer.open()
        }
        function onLogAdded(payload) {
            let row = JSON.parse(payload)
            logModel.append(row)
            if (logModel.count > 1000) logModel.remove(0, logModel.count - 1000)
        }
        function onRecordingsChanged(payload) {
            let data = JSON.parse(payload)
            recordingModel.clear()
            for (let row of data.items) recordingModel.append(row)
            syncStatus.text = data.sync_active ? "Drive timer active" : "Drive timer inactive"
        }
    }

    ListModel { id: preflightModel }
    ListModel { id: logModel }
    ListModel { id: recordingModel }

    component Panel: Rectangle {
        color: "#111A24"
        radius: 14
        border.color: "#203040"
        border.width: 1
    }

    component SectionTitle: Label {
        color: "#E8F0F7"
        font.pixelSize: 19
        font.weight: Font.DemiBold
    }

    component MutedText: Label {
        color: "#8FA5B8"
        font.pixelSize: 13
        wrapMode: Text.WordWrap
    }

    component CalmButton: Button {
        id: control
        implicitHeight: 42
        leftPadding: 18
        rightPadding: 18
        font.pixelSize: 14
        font.weight: Font.DemiBold
        background: Rectangle {
            radius: 9
            color: control.enabled ? (control.down ? "#227F88" : "#193842") : "#17212B"
            border.color: control.enabled ? "#35C6D0" : "#293641"
        }
        contentItem: Text {
            text: control.text
            color: control.enabled ? "#DDFBFD" : "#637381"
            font: control.font
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    component SafetyButton: Button {
        id: control
        implicitHeight: 46
        font.pixelSize: 14
        font.weight: Font.Bold
        background: Rectangle {
            radius: 10
            color: control.down ? "#7D252C" : "#4A2027"
            border.color: "#E15C68"
        }
        contentItem: Text {
            text: control.text
            color: "#FFE8EA"
            font: control.font
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    component StatusDot: Rectangle {
        property bool healthy: false
        width: 9; height: 9; radius: 5
        color: healthy ? "#43D19E" : "#667788"
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        Rectangle {
            Layout.preferredWidth: 232
            Layout.fillHeight: true
            color: "#0E1620"
            border.color: "#192634"

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 18
                spacing: 8

                RowLayout {
                    Layout.bottomMargin: 26
                    Rectangle {
                        width: 38; height: 38; radius: 11
                        color: "#173840"
                        Text { anchors.centerIn: parent; text: "D"; color: "#55DCE5"; font.pixelSize: 21; font.bold: true }
                    }
                    ColumnLayout {
                        Label { text: "DracoViLoc"; color: "#F0F7FA"; font.pixelSize: 17; font.bold: true }
                        Label { text: "CONTROL CENTER"; color: "#5F7C8E"; font.pixelSize: 9; font.letterSpacing: 1.6 }
                    }
                }

                Repeater {
                    model: ["Dashboard", "Run setup", "Recordings", "Logs", "System"]
                    delegate: Button {
                        id: navButton
                        required property string modelData
                        required property int index
                        Layout.fillWidth: true
                        implicitHeight: 44
                        text: modelData
                        flat: true
                        onClicked: {
                            root.currentPage = index
                            if (index === 2) root.controller.refreshRecordings()
                        }
                        background: Rectangle {
                            radius: 9
                            color: root.currentPage === index ? "#17313A" : "transparent"
                            border.color: root.currentPage === index ? "#28545C" : "transparent"
                        }
                        contentItem: Text {
                            text: navButton.text
                            color: root.currentPage === index ? "#67DFE7" : "#93A7B8"
                            font.pixelSize: 14
                            font.weight: root.currentPage === index ? Font.DemiBold : Font.Normal
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 12
                        }
                    }
                }
                Item { Layout.fillHeight: true }

                Panel {
                    Layout.fillWidth: true
                    implicitHeight: 106
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 13; spacing: 5
                        RowLayout {
                            StatusDot { healthy: root.controller.runActive }
                            Label { text: root.controller.state; color: "#D9E6EE"; font.bold: true }
                        }
                        MutedText { text: "ROS domain 42 · UDPv4" }
                        MutedText { text: "Local control only" }
                    }
                }
            }
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 76
                color: "#0B1118"
                border.color: "#192634"
                RowLayout {
                    anchors.fill: parent; anchors.leftMargin: 26; anchors.rightMargin: 26
                    ColumnLayout {
                        Label { text: ["Operations", "Configure run", "Recording library", "System logs", "System readiness"][root.currentPage]; color: "#EEF5F8"; font.pixelSize: 22; font.bold: true }
                        MutedText { text: root.controller.message; Layout.maximumWidth: 760 }
                    }
                    Item { Layout.fillWidth: true }
                    SafetyButton {
                        text: "STOP MOTION"
                        Layout.preferredWidth: 154
                        enabled: root.controller.runActive && root.controller.state !== "Shutting Down"
                        onClicked: root.controller.stopMotion()
                    }
                }
            }

            StackLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentIndex: root.currentPage

                // Dashboard
                ScrollView {
                    contentWidth: availableWidth
                    ColumnLayout {
                        width: parent.width; spacing: 16
                        anchors.margins: 22
                        RowLayout {
                            Layout.fillWidth: true; spacing: 16
                            Panel {
                                Layout.fillWidth: true; Layout.preferredWidth: 760; Layout.minimumWidth: 480; Layout.preferredHeight: 455
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 16
                                    RowLayout {
                                        SectionTitle { text: "Live camera" }
                                        Item { Layout.fillWidth: true }
                                        StatusDot { healthy: root.liveStatus.freshness !== undefined && root.liveStatus.freshness["/image"] !== undefined && root.liveStatus.freshness["/image"] < 1.0 }
                                        MutedText { text: root.liveStatus.freshness !== undefined && root.liveStatus.freshness["/image"] !== undefined && root.liveStatus.freshness["/image"] < 1.0 ? "Live" : "Waiting" }
                                    }
                                    Rectangle {
                                        Layout.fillWidth: true; Layout.fillHeight: true
                                        radius: 10; color: "#080D12"; clip: true
                                        Image {
                                            anchors.fill: parent; anchors.margins: 2
                                            source: root.cameraRevision > 0 ? "image://camera/live?" + root.cameraRevision : ""
                                            fillMode: Image.PreserveAspectFit
                                            cache: false
                                        }
                                        Column {
                                            anchors.centerIn: parent
                                            visible: !(root.liveStatus.freshness !== undefined && root.liveStatus.freshness["/image"] !== undefined && root.liveStatus.freshness["/image"] < 1.0)
                                            spacing: 8
                                            Label { anchors.horizontalCenter: parent.horizontalCenter; text: "CAMERA STANDBY"; color: "#617586"; font.pixelSize: 14; font.bold: true }
                                            MutedText { text: "Start a YOLO profile to receive /image" }
                                        }
                                    }
                                }
                            }
                            ColumnLayout {
                                Layout.preferredWidth: 355; Layout.minimumWidth: 355; Layout.maximumWidth: 390; spacing: 16
                                Panel {
                                    Layout.fillWidth: true; Layout.preferredHeight: 212
                                    ColumnLayout {
                                        anchors.fill: parent; anchors.margins: 16; spacing: 10
                                        SectionTitle { text: "Run state" }
                                        Label { text: root.controller.state; color: "#55DCE5"; font.pixelSize: 25; font.bold: true }
                                        MutedText { text: root.profile.name }
                                        RowLayout {
                                            CalmButton { text: "Preflight"; enabled: !root.controller.runActive; onClicked: root.controller.runPreflight() }
                                            CalmButton { text: "Start run"; enabled: !root.controller.runActive; onClicked: root.controller.startRun() }
                                        }
                                        CalmButton { Layout.fillWidth: true; text: "Stop all safely"; enabled: root.controller.runActive; onClicked: stopAllDialog.open() }
                                        SafetyButton {
                                            Layout.fillWidth: true
                                            visible: root.controller.state === "Fault" && root.controller.runActive && root.profile.hardware_mode === "real"
                                            text: "E-stop engaged · force shutdown"
                                            onClicked: forceStopDialog.open()
                                        }
                                    }
                                }
                                Panel {
                                    Layout.fillWidth: true; Layout.preferredHeight: 227
                                    ColumnLayout {
                                        anchors.fill: parent; anchors.margins: 16; spacing: 10
                                        SectionTitle { text: "Motion control" }
                                        MutedText { Layout.fillWidth: true; text: "Motion never starts with the run. Home first, then arm tracking." }
                                        DelayButton {
                                            Layout.fillWidth: true; delay: 1500
                                            text: "Hold to home · 1.5 s"
                                            enabled: root.controller.state === "Ready" || root.controller.state === "Home Ready"
                                            onActivated: homeDialog.open()
                                        }
                                        DelayButton {
                                            Layout.fillWidth: true; delay: 1500
                                            text: "Hold to enable tracking · 1.5 s"
                                            enabled: root.controller.state === "Home Ready" || root.controller.state === "Ready"
                                            onActivated: trackingDialog.open()
                                        }
                                        MutedText { Layout.fillWidth: true; text: "Software stop is not a physical emergency stop."; color: "#D8A65A" }
                                    }
                                }
                            }
                        }

                        Panel {
                            Layout.fillWidth: true; Layout.preferredHeight: 155
                            RowLayout {
                                anchors.fill: parent; anchors.margins: 18; spacing: 32
                                ColumnLayout {
                                    SectionTitle { text: "ROS health" }
                                    MutedText { text: "Joint feedback" }
                                    Label { text: (root.liveStatus.joint_count || 0) + " joints"; color: "#DDEAF1"; font.pixelSize: 18 }
                                }
                                Repeater {
                                    model: ["/yolo/direction", "/mobilenetv2/direction", "/ekf/direction"]
                                    delegate: ColumnLayout {
                                        id: directionStatus
                                        required property string modelData
                                        property real age: root.liveStatus.freshness !== undefined && root.liveStatus.freshness[modelData] !== undefined ? root.liveStatus.freshness[modelData] : 999
                                        MutedText { text: directionStatus.modelData }
                                        RowLayout {
                                            StatusDot { healthy: directionStatus.age < 1.5 }
                                            Label { text: directionStatus.age < 1.5 ? "Fresh" : (directionStatus.age < 999 ? "Stale" : "Missing"); color: directionStatus.age < 1.5 ? "#69D9B1" : "#A3B2BE" }
                                        }
                                    }
                                }
                                Item { Layout.fillWidth: true }
                            }
                        }
                        Panel {
                            Layout.fillWidth: true; Layout.preferredHeight: 185
                            RowLayout {
                                anchors.fill: parent; anchors.margins: 18; spacing: 24
                                ColumnLayout {
                                    Layout.preferredWidth: 220
                                    SectionTitle { text: "Direction cue" }
                                    MutedText { text: "EKF preferred, then YOLO or MobileNetV2" }
                                    Label {
                                        property var directions: root.liveStatus.directions || ({})
                                        property var vector: directions["/ekf/direction"] || directions["/yolo/direction"] || directions["/mobilenetv2/direction"]
                                        text: vector ? ("Azimuth " + (Math.atan2(vector.y, vector.x) * 180 / Math.PI).toFixed(1) + "°") : "No current bearing"
                                        color: vector ? "#55DCE5" : "#718595"
                                        font.pixelSize: 17; font.bold: true
                                    }
                                }
                                Canvas {
                                    id: directionCompass
                                    Layout.preferredWidth: 130; Layout.preferredHeight: 130
                                    property var directions: root.liveStatus.directions || ({})
                                    property var vector: directions["/ekf/direction"] || directions["/yolo/direction"] || directions["/mobilenetv2/direction"]
                                    onVectorChanged: requestPaint()
                                    onPaint: {
                                        let ctx = getContext("2d")
                                        ctx.reset()
                                        let cx = width / 2, cy = height / 2, radius = Math.min(width, height) * 0.42
                                        ctx.strokeStyle = "#304454"; ctx.lineWidth = 2
                                        ctx.beginPath(); ctx.arc(cx, cy, radius, 0, Math.PI * 2); ctx.stroke()
                                        ctx.fillStyle = "#718595"; ctx.font = "11px sans-serif"; ctx.fillText("FRONT", cx - 18, 12)
                                        if (vector) {
                                            let angle = Math.atan2(vector.y, vector.x)
                                            let ex = cx + Math.cos(angle) * radius * 0.82
                                            let ey = cy - Math.sin(angle) * radius * 0.82
                                            ctx.strokeStyle = "#55DCE5"; ctx.lineWidth = 5; ctx.lineCap = "round"
                                            ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke()
                                            ctx.fillStyle = "#55DCE5"; ctx.beginPath(); ctx.arc(ex, ey, 6, 0, Math.PI * 2); ctx.fill()
                                        }
                                    }
                                }
                                ColumnLayout {
                                    Layout.fillWidth: true
                                    SectionTitle { text: "Controllers" }
                                    MutedText { text: root.liveStatus.controllers ? JSON.stringify(root.liveStatus.controllers).replace(/[{}\"]/g, "").replace(/,/g, "  ·  ") : "Waiting for controller_manager"; Layout.fillWidth: true }
                                    MutedText { text: "Audio: " + (root.liveStatus.freshness && root.liveStatus.freshness["/sss"] < 1.0 ? "fresh" : "waiting") + "   ·   Detections: " + (root.liveStatus.freshness && root.liveStatus.freshness["/detections_output"] < 1.0 ? "fresh" : "waiting") }
                                }
                            }
                        }
                    }
                }

                // Run setup
                ScrollView {
                    contentWidth: availableWidth
                    ColumnLayout {
                        width: parent.width; spacing: 16
                        anchors.margins: 22
                        Panel {
                            Layout.fillWidth: true; Layout.preferredHeight: 110
                            RowLayout {
                                anchors.fill: parent; anchors.margins: 18; spacing: 14
                                ColumnLayout {
                                    SectionTitle { text: "Operating profile" }
                                    MutedText { text: "Profiles restore configuration only—never motion." }
                                }
                                Item { Layout.fillWidth: true }
                                ComboBox {
                                    id: profileBox; Layout.preferredWidth: 290
                                    model: root.controller.profileNames
                                    enabled: !root.controller.runActive
                                    onActivated: root.controller.selectProfile(currentText)
                                }
                                TextField { id: saveProfileName; Layout.preferredWidth: 180; placeholderText: "Custom profile name"; enabled: !root.controller.runActive }
                                CalmButton { text: "Save"; enabled: !root.controller.runActive && saveProfileName.text.length > 0; onClicked: root.controller.saveProfile(saveProfileName.text) }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 16
                            Panel {
                                Layout.fillWidth: true; Layout.preferredHeight: 330
                                GridLayout {
                                    anchors.fill: parent; anchors.margins: 18; columns: 2; columnSpacing: 24; rowSpacing: 13
                                    SectionTitle { text: "Pipeline"; Layout.columnSpan: 2 }
                                    CheckBox { text: "Audio / ODAS"; checked: root.profile.audio_enabled; enabled: !root.controller.runActive; onToggled: root.controller.setOption("audio_enabled", checked) }
                                    CheckBox { text: "Isaac YOLO"; checked: root.profile.yolo_enabled; enabled: !root.controller.runActive; onToggled: root.controller.setOption("yolo_enabled", checked) }
                                    CheckBox { text: "MobileNetV2"; checked: root.profile.mobilenetv2_enabled; enabled: !root.controller.runActive; onToggled: root.controller.setOption("mobilenetv2_enabled", checked) }
                                    CheckBox { text: "AST"; checked: root.profile.ast_enabled; enabled: !root.controller.runActive; onToggled: root.controller.setOption("ast_enabled", checked) }
                                    CheckBox { text: "GRE"; checked: root.profile.gre_enabled; enabled: !root.controller.runActive; onToggled: root.controller.setOption("gre_enabled", checked) }
                                    CheckBox { text: "EKF fusion"; checked: root.profile.fusion_enabled; enabled: !root.controller.runActive; onToggled: root.controller.setOption("fusion_enabled", checked) }
                                    CheckBox { text: "Record run"; checked: root.profile.recording_enabled; enabled: !root.controller.runActive; onToggled: root.controller.setOption("recording_enabled", checked) }
                                    CheckBox { text: "Diagnostics"; checked: root.profile.diagnostics_enabled; enabled: !root.controller.runActive; onToggled: root.controller.setOption("diagnostics_enabled", checked) }
                                }
                            }
                            Panel {
                                Layout.fillWidth: true; Layout.preferredHeight: 330
                                GridLayout {
                                    anchors.fill: parent; anchors.margins: 18; columns: 2; columnSpacing: 18; rowSpacing: 12
                                    SectionTitle { text: "Robot and tracking"; Layout.columnSpan: 2 }
                                    MutedText { text: "Hardware" }
                                    ComboBox { model: ["mock", "real"]; currentIndex: root.profile.hardware_mode === "real" ? 1 : 0; enabled: !root.controller.runActive; onActivated: root.controller.setOption("hardware_mode", currentText) }
                                    MutedText { text: "Robot IP" }
                                    TextField { text: root.profile.robot_ip; enabled: !root.controller.runActive; onEditingFinished: root.controller.setOption("robot_ip", text) }
                                    MutedText { text: "Tracking source" }
                                    ComboBox { id: modeBox; model: ["off", "direct_yolo", "direct_mobilenetv2", "direct_ast", "direct_gre", "direct_either", "ekf"]; currentIndex: model.indexOf(root.profile.tracking_mode); onActivated: root.controller.setOption("tracking_mode", currentText) }
                                    MutedText { text: "Use RViz" }
                                    Switch { checked: root.profile.use_rviz; onToggled: root.controller.setOption("use_rviz", checked) }
                                    MutedText { text: "MoveIt" }
                                    Switch { checked: root.profile.use_moveit; onToggled: root.controller.setOption("use_moveit", checked) }
                                }
                            }
                        }
                        Panel {
                            Layout.fillWidth: true; Layout.preferredHeight: 250
                            GridLayout {
                                anchors.fill: parent; anchors.margins: 18; columns: 4; columnSpacing: 18; rowSpacing: 10
                                SectionTitle { text: "Expert motion limits"; Layout.columnSpan: 4 }
                                Repeater {
                                    model: [
                                        {label:"Velocity", key:"max_velocity"}, {label:"Acceleration", key:"max_acceleration"},
                                        {label:"Jerk", key:"max_jerk"}, {label:"Tracking error", key:"max_tracking_error"},
                                        {label:"Smoothing", key:"smoothing_alpha"}, {label:"Deadband", key:"angular_deadband"},
                                        {label:"EKF average", key:"ekf_average_window"}, {label:"Command rate", key:"command_rate_hz"}
                                    ]
                                    delegate: ColumnLayout {
                                        required property var modelData
                                        MutedText { text: modelData.label }
                                        TextField { text: root.profile[modelData.key]; enabled: !root.controller.runActive; onEditingFinished: root.controller.setOption(modelData.key, text) }
                                    }
                                }
                            }
                        }
                        Panel {
                            Layout.fillWidth: true; Layout.preferredHeight: 165
                            RowLayout {
                                anchors.fill: parent; anchors.margins: 18; spacing: 12
                                ColumnLayout {
                                    Layout.preferredWidth: 260
                                    SectionTitle { text: "All launch overrides" }
                                    MutedText { text: "Search any argument declared by arm_demo.launch.py and apply a string override." }
                                }
                                ComboBox {
                                    id: expertName
                                    Layout.preferredWidth: 270
                                    editable: true
                                    model: root.controller.launchArgumentNames
                                }
                                TextField { id: expertValue; Layout.fillWidth: true; placeholderText: "Override value" }
                                CalmButton {
                                    text: "Apply"
                                    enabled: !root.controller.runActive && expertName.editText.length > 0 && expertValue.text.length > 0
                                    onClicked: root.controller.setExpertOption(expertName.editText, expertValue.text)
                                }
                            }
                        }
                    }
                }

                // Recordings
                ColumnLayout {
                    spacing: 14; anchors.margins: 22
                    RowLayout {
                        SectionTitle { text: "Completed and active sessions" }
                        Item { Layout.fillWidth: true }
                        MutedText { id: syncStatus; text: "Drive status unknown" }
                        CalmButton { text: "Sync now"; onClicked: root.controller.syncNow() }
                        CalmButton { text: "Refresh"; onClicked: root.controller.refreshRecordings() }
                    }
                    Panel {
                        Layout.fillWidth: true; Layout.fillHeight: true
                        ListView {
                            anchors.fill: parent; anchors.margins: 10; clip: true; spacing: 6
                            model: recordingModel
                            delegate: Rectangle {
                                required property string timestamp
                                required property string kind
                                required property string path
                                required property string state
                                required property real size_bytes
                                width: ListView.view.width; height: 64; radius: 9; color: "#151F2A"
                                RowLayout {
                                    anchors.fill: parent; anchors.margins: 12
                                    Label { text: kind.toUpperCase(); color: kind === "video" ? "#55DCE5" : "#A7D985"; font.bold: true; Layout.preferredWidth: 70 }
                                    ColumnLayout {
                                        Layout.fillWidth: true
                                        Label { text: timestamp; color: "#E0EBF1"; font.bold: true }
                                        MutedText { text: path; elide: Text.ElideMiddle; Layout.fillWidth: true }
                                    }
                                    Label { text: (size_bytes / 1048576).toFixed(1) + " MB"; color: "#98ABBA" }
                                    Label { text: state; color: state === "complete" ? "#63D8AA" : "#E3B466"; Layout.preferredWidth: 90 }
                                    CalmButton { text: "Open"; onClicked: root.controller.openPath(path) }
                                }
                            }
                        }
                    }
                }

                // Logs
                ColumnLayout {
                    anchors.margins: 22; spacing: 12
                    RowLayout {
                        SectionTitle { text: "Unified runtime log" }
                        Item { Layout.fillWidth: true }
                        MutedText { text: logModel.count + " events" }
                        CalmButton { text: "Clear"; onClicked: logModel.clear() }
                    }
                    Panel {
                        Layout.fillWidth: true; Layout.fillHeight: true
                        ListView {
                            anchors.fill: parent; anchors.margins: 8; clip: true; model: logModel; spacing: 2
                            delegate: Rectangle {
                                required property string timestamp
                                required property string subsystem
                                required property string severity
                                required property string message
                                width: ListView.view.width; height: Math.max(36, logText.implicitHeight + 14); color: index % 2 ? "#111A24" : "#131E29"
                                RowLayout {
                                    anchors.fill: parent; anchors.margins: 7
                                    Label { text: timestamp.slice(11); color: "#62798B"; font.family: "monospace"; Layout.preferredWidth: 72 }
                                    Label { text: subsystem; color: "#57CAD3"; font.bold: true; Layout.preferredWidth: 90 }
                                    Label { id: logText; text: message; color: severity === "error" ? "#FF8992" : "#C8D7E0"; wrapMode: Text.Wrap; Layout.fillWidth: true }
                                }
                            }
                        }
                    }
                }

                // System
                ScrollView {
                    contentWidth: availableWidth
                    ColumnLayout {
                        width: parent.width; spacing: 16; anchors.margins: 22
                        Panel {
                            Layout.fillWidth: true; Layout.preferredHeight: 160
                            RowLayout {
                                anchors.fill: parent; anchors.margins: 20
                                ColumnLayout {
                                    SectionTitle { text: "System readiness" }
                                    MutedText { text: "T2000 desktop profile · ROS Humble · domain 42" }
                                    MutedText { text: "Run preflight after connecting the UMA16, camera, and robot." }
                                }
                                Item { Layout.fillWidth: true }
                                CalmButton { text: "Run full preflight"; enabled: !root.controller.runActive; onClicked: root.controller.runPreflight() }
                            }
                        }
                        Panel {
                            Layout.fillWidth: true; Layout.preferredHeight: 220
                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 20
                                SectionTitle { text: "Safety boundary" }
                                MutedText { text: "This application can request controlled ROS motion stops. It is not safety-rated and cannot replace the FAIRINO emergency stop, controller safeguards, or a clear workspace." }
                                Rectangle { Layout.fillWidth: true; height: 1; color: "#263746" }
                                MutedText { text: "The application will not automatically home or enable tracking. Real-arm shutdown is blocked if a controlled hold cannot be confirmed."; color: "#D8A65A" }
                            }
                        }
                    }
                }
            }
        }
    }

    Drawer {
        id: preflightDrawer
        edge: Qt.RightEdge
        width: Math.min(root.width * 0.42, 570)
        height: root.height
        background: Rectangle { color: "#101923"; border.color: "#29404F" }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 20; spacing: 14
            SectionTitle { text: "Preflight report" }
            MutedText { text: "Blocking failures must be resolved before this profile can start." }
            ListView {
                Layout.fillWidth: true; Layout.fillHeight: true; model: preflightModel; spacing: 7; clip: true
                delegate: Rectangle {
                    required property string label
                    required property bool ok
                    required property string detail
                    required property bool blocking
                    width: ListView.view.width; height: 68; radius: 9; color: "#16222D"
                    RowLayout {
                        anchors.fill: parent; anchors.margins: 11
                        Rectangle { width: 12; height: 12; radius: 6; color: ok ? "#43D19E" : (blocking ? "#E15C68" : "#E3B466") }
                        ColumnLayout {
                            Layout.fillWidth: true
                            Label { text: label; color: "#E3EDF3"; font.bold: true }
                            MutedText { text: detail; Layout.fillWidth: true }
                        }
                    }
                }
            }
            CalmButton { Layout.fillWidth: true; text: "Close"; onClicked: preflightDrawer.close() }
        }
    }

    Dialog {
        id: homeDialog; modal: true; anchors.centerIn: parent; title: "Confirm homing"
        width: 500
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: root.controller.requestHome()
        contentItem: MutedText { text: "Confirm the workspace is clear and the physical emergency stop is within reach. The arm will move to its home pose."; width: 430 }
    }
    Dialog {
        id: trackingDialog; modal: true; anchors.centerIn: parent; title: "Enable tracking"
        width: 500
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: root.controller.enableTracking()
        contentItem: MutedText { text: "Tracking will wait for a fresh detector direction, then command joints 1 and 4. Keep the workspace clear."; width: 430 }
    }
    Dialog {
        id: stopAllDialog; modal: true; anchors.centerIn: parent; title: "Stop the complete run?"
        width: 500
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: root.controller.stopAll()
        contentItem: MutedText { text: "Tracking will stop first. Recorders will then finalize before the host stack and YOLO container are stopped."; width: 430 }
    }
    Dialog {
        id: closeDialog; modal: true; anchors.centerIn: parent; title: "Close control center?"
        width: 500
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: root.controller.requestQuit()
        contentItem: MutedText { text: "An active run is managed by this window. Confirm to stop motion, finalize recordings, shut down managed processes, and exit."; width: 430 }
    }
    Dialog {
        id: forceStopDialog; modal: true; anchors.centerIn: parent; title: "Confirm physical emergency stop"
        width: 520
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: root.controller.forceStopAfterEstop()
        contentItem: MutedText { text: "Only continue if the physical FAIRINO emergency stop is engaged and motion is mechanically safe. This will terminate the control processes without a confirmed ROS hold."; width: 450; color: "#FF9AA2" }
    }
}
