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
        property string remoteJson: "{\"state\":\"disconnected\",\"has_control\":false}"
        property bool remoteControllable: false
        property string processesJson: "[]"
        property string sessionLogDirectory: ""
        property string runDirectory: ""
        property string processListJson: "{\"items\": [], \"report\": []}"
        property string robotCheckJson: "{}"
        function refreshProcesses() {}
        function stopProcessItem(id, force) {}
        function restartUnit(name) {}
        function openRviz() {}
        function cleanUpEverything() {}
        function checkRobots() {}
        function removeExpertOption(name) {}
        property bool rvizEmbedded: false
        property string arrowsJson: "{}"
        function setArrowVisible(name, shown) {}
        function setRvizArea(x, y, w, h, visible) {}
        property string rvizStatus: ""
        function setThreeDView(on) {}
        property string setupJson: "{\"jobs\": [], \"saved_images\": []}"
        property bool jobActive: false
        function refreshSetup() {}
        function runJob(id) {}
        function updateAll() {}
        function cancelJob() {}
        function saveProjectImage() {}
        function loadProjectImage(name) {}
        function exportModel(id) {}
        function setJobOption(id, name, value) {}
        function buildModelPlan(id) {}
        function terminalModel(name) { return null }
        function uiEvent(action, detail) {}
        function openSessionFolder() {}
        function openProcessLog(name) {}
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
        function connectRemote() {}
        function disconnectRemote() {}
        function acquireRemoteControl() {}
        function deployRemoteAgent() {}
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
    property var cameraStreams: []
    property string cameraView: "overlay"   // raw | overlay | both
    property var telemetry: ({})
    property var history: ({"score": [], "nis": [], "ee": [], "latency": []})
    readonly property int historyLength: 150   // 15 s at 10 Hz
    function fmt(value, digits) {
        return (value === null || value === undefined || isNaN(value)) ? "–" : Number(value).toFixed(digits === undefined ? 1 : digits)
    }
    function cameraSource(stream) {
        if (root.cameraRevision <= 0 || root.cameraStreams.indexOf(stream) < 0) return ""
        return "image://camera/" + stream + "?" + root.cameraRevision
    }
    function stopItem(id, name, force) {
        root.controller.stopProcessItem(id, force)
    }
    function pushHistory(name, value) {
        let series = root.history[name].slice(-(root.historyLength - 1))
        series.push(value === null || value === undefined ? NaN : value)
        let next = Object.assign({}, root.history)
        next[name] = series
        root.history = next
    }
    property bool shutdownApproved: false
    property var profile: JSON.parse(root.controller.profileJson)
    property var liveStatus: ({})
    property var remoteState: JSON.parse(root.controller.remoteJson)
    property var processes: JSON.parse(root.controller.processesJson)
    property string selectedTerminal: ""
    property var setup: JSON.parse(root.controller.setupJson)
    property var processList: JSON.parse(root.controller.processListJson)
    property var robotCheck: JSON.parse(root.controller.robotCheckJson)
    readonly property var pageNames: ["Dashboard", "Perception", "Run setup", "Recordings", "Logs", "Processes", "Setup & builds", "System"]
    readonly property var pageTitles: ["Operations", "Perception", "Configure run", "Recording library", "System logs", "Processes", "Setup & builds", "System readiness"]
    property string setupTerminal: ""
    readonly property string visualName: ({ yolo: "YOLO", dfine: "D-FINE", dfine_n: "D-FINE-N" })[root.profile.visual_detector || "yolo"] || "YOLO"

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
        ignoreUnknownSignals: true
        function onTelemetryUpdated(payload) {
            let t = JSON.parse(payload)
            root.telemetry = t
            pushHistory("score", t.yolo ? t.yolo.best_score : NaN)
            pushHistory("latency", t.yolo ? t.yolo.latency_ms : NaN)
            pushHistory("ee", t.ee ? t.ee.speed : NaN)
            let nis = NaN
            if (t.ekf && t.ekf.sources) {
                let newest = 1e9
                for (let name in t.ekf.sources) {
                    let source = t.ekf.sources[name]
                    if (source.age_s !== null && source.age_s < newest) { newest = source.age_s; nis = source.last_nis }
                }
            }
            pushHistory("nis", nis)
        }
        function onProfileChanged() { root.profile = JSON.parse(root.controller.profileJson) }
        function onRemoteChanged() { root.remoteState = JSON.parse(root.controller.remoteJson) }
        function onProcessesChanged() { root.processes = JSON.parse(root.controller.processesJson) }
        function onProcessListChanged() { root.processList = JSON.parse(root.controller.processListJson) }
        function onRobotCheckChanged() { root.robotCheck = JSON.parse(root.controller.robotCheckJson) }
        function onSetupChanged() {
            root.setup = JSON.parse(root.controller.setupJson)
            if (root.setup.current) root.setupTerminal = "job-" + root.setup.current
        }
        function onStatusChanged(payload) { root.liveStatus = JSON.parse(payload) }
        function onPreflightFinished(payload, ok) {
            preflightModel.clear()
            let rows = JSON.parse(payload)
            for (let row of rows) preflightModel.append(row)
            // A passing preflight is visible in the status message; only
            // failures need the full report in front of the operator.
            if (!ok || root.pageNames[root.currentPage] === "System") preflightDrawer.open()
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

    component PrimaryButton: Button {
        id: primary
        implicitHeight: 42
        leftPadding: 18
        rightPadding: 18
        font.pixelSize: 14
        font.weight: Font.DemiBold
        background: Rectangle {
            radius: 9
            color: primary.enabled ? (primary.down ? "#2AA3AB" : "#35C6D0") : "#17212B"
            border.color: primary.enabled ? "#67DFE7" : "#293641"
        }
        contentItem: Text {
            text: primary.text
            color: primary.enabled ? "#071014" : "#637381"
            font: primary.font
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }
    component Sparkline: Canvas {
        id: spark
        property var values: []
        property color lineColor: "#57CAD3"
        property real threshold: NaN
        implicitHeight: 54
        onValuesChanged: requestPaint()
        onPaint: {
            let ctx = getContext("2d")
            ctx.reset()
            let points = values.filter(v => !isNaN(v))
            if (points.length < 2) return
            let low = Math.min.apply(null, points), high = Math.max.apply(null, points)
            if (!isNaN(threshold)) { low = Math.min(low, threshold); high = Math.max(high, threshold) }
            if (high - low < 1e-9) { high += 1; low -= 1 }
            let y = v => height - 3 - (v - low) / (high - low) * (height - 6)
            if (!isNaN(threshold)) {
                ctx.strokeStyle = "#7A5A2A"; ctx.lineWidth = 1
                ctx.beginPath(); ctx.moveTo(0, y(threshold)); ctx.lineTo(width, y(threshold)); ctx.stroke()
            }
            ctx.strokeStyle = lineColor; ctx.lineWidth = 1.6
            ctx.beginPath()
            let started = false
            for (let i = 0; i < values.length; ++i) {
                if (isNaN(values[i])) { started = false; continue }
                let x = i / Math.max(1, root.historyLength - 1) * width
                if (!started) { ctx.moveTo(x, y(values[i])); started = true } else ctx.lineTo(x, y(values[i]))
            }
            ctx.stroke()
        }
    }
    component Metric: ColumnLayout {
        property string label
        property string value
        property color valueColor: "#EEF5F8"
        spacing: 1
        Label { text: parent.label; color: "#7E8F9C"; font.pixelSize: 11 }
        Label { text: parent.value; color: parent.valueColor; font.pixelSize: 17; font.bold: true; font.family: "monospace" }
    }
    component CameraPane: Rectangle {
        property string stream: "raw"
        radius: 10; color: "#080D12"; clip: true
        Image {
            anchors.fill: parent; anchors.margins: 2
            source: root.cameraSource(parent.stream)
            fillMode: Image.PreserveAspectFit
            cache: false
        }
        Column {
            anchors.centerIn: parent; spacing: 6
            visible: root.cameraSource(parent.stream) === ""
            Label { anchors.horizontalCenter: parent.horizontalCenter; text: parent.parent.stream === "raw" ? "RAW CAMERA STANDBY" : "OVERLAY STANDBY"; color: "#617586"; font.pixelSize: 13; font.bold: true }
            MutedText { anchors.horizontalCenter: parent.horizontalCenter; text: "Waiting for /dracoviloc/preview/" + parent.parent.stream }
        }
        Label {
            anchors.left: parent.left; anchors.top: parent.top; anchors.margins: 8
            text: parent.stream === "raw" ? "RAW" : "OVERLAY"; color: "#93A7B8"; font.pixelSize: 10; font.bold: true
        }
    }
    component ViewSwitch: RowLayout {
        spacing: 4
        Repeater {
            model: [["raw", "Raw"], ["overlay", "Overlay"], ["both", "Side by side"]]
            delegate: Button {
                required property var modelData
                flat: true; implicitHeight: 30
                text: modelData[1]
                onClicked: { root.cameraView = modelData[0]; root.controller.uiEvent("camera_view", modelData[0]) }
                background: Rectangle { radius: 7; color: root.cameraView === parent.modelData[0] ? "#17313A" : "#121C26"; border.color: root.cameraView === parent.modelData[0] ? "#35C6D0" : "#1E2C39" }
                contentItem: Text { text: parent.text; color: root.cameraView === parent.modelData[0] ? "#67DFE7" : "#93A7B8"; font.pixelSize: 12; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; leftPadding: 8; rightPadding: 8 }
            }
        }
    }
    component HoldButton: DelayButton {
        id: hold
        implicitHeight: 42
        leftPadding: 18
        rightPadding: 18
        delay: 1000
        font.pixelSize: 14
        font.weight: Font.DemiBold
        background: Rectangle {
            radius: 9
            color: hold.enabled ? "#193842" : "#17212B"
            border.color: hold.enabled ? "#35C6D0" : "#293641"
            clip: true
            Rectangle {  // fills while the button is held
                width: parent.width * hold.progress; height: parent.height
                radius: 9; color: "#2AA3AB"; opacity: 0.55
            }
        }
        contentItem: Text {
            text: hold.text
            color: hold.enabled ? "#DDFBFD" : "#637381"
            font: hold.font
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }
    component SafetyButton: Button {
        id: control
        implicitHeight: 46
        leftPadding: 16
        rightPadding: 16
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
                        width: 46; height: 46; radius: 11
                        color: "#FFFFFF"
                        clip: true
                        Image {
                            anchors.fill: parent
                            anchors.margins: 3
                            source: Qt.resolvedUrl("../assets/dracoviloc-logo.png")
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                            mipmap: true
                        }
                    }
                    ColumnLayout {
                        Label { text: "DracoViLoc"; color: "#F0F7FA"; font.pixelSize: 17; font.bold: true }
                        Label { text: "CONTROL CENTER"; color: "#5F7C8E"; font.pixelSize: 9; font.letterSpacing: 1.6 }
                    }
                }

                Repeater {
                    model: root.pageNames
                    delegate: Button {
                        id: navButton
                        required property string modelData
                        required property int index
                        Layout.fillWidth: true
                        implicitHeight: 44
                        text: modelData
                        flat: true
                        onClicked: {
                            root.controller.uiEvent("navigate", modelData)
                            root.currentPage = index
                            if (modelData === "Recordings") root.controller.refreshRecordings()
                            if (modelData === "Run setup" || modelData === "Setup & builds") root.controller.refreshSetup()
                            if (modelData === "Processes") root.controller.refreshProcesses()
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
                        MutedText { text: root.profile.execution_target === "remote" ? ("Jetson · " + (root.remoteState.state || "disconnected")) : "Local control" }
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
                        Label { text: root.pageTitles[root.currentPage]; color: "#EEF5F8"; font.pixelSize: 22; font.bold: true }
                        MutedText { text: root.controller.message; Layout.maximumWidth: 760 }
                    }
                    Item { Layout.fillWidth: true }
                    SafetyButton {
                        text: "STOP MOTION"
                        Layout.preferredWidth: 154
                        enabled: root.controller.runActive && root.controller.state !== "Shutting Down" && (root.profile.execution_target !== "remote" || root.controller.remoteControllable)
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
                    id: dashboardScroll
                    contentWidth: availableWidth
                    ColumnLayout {
                        x: 22; width: parent.width - 44; spacing: 16
                        Item { Layout.preferredHeight: 6 }
                        Panel {
                            Layout.fillWidth: true; Layout.preferredHeight: 92
                            RowLayout {
                                anchors.fill: parent; anchors.leftMargin: 16; anchors.rightMargin: 16; spacing: 10
                                ColumnLayout {
                                    spacing: 0; Layout.preferredWidth: 150
                                    Label { text: root.controller.state; color: "#55DCE5"; font.pixelSize: 20; font.bold: true }
                                    MutedText { text: (root.profile.hardware_mode === "real" ? "REAL" : "MOCK") + (root.profile.dual_arm ? " · 2 cobots" : " · 1 cobot"); color: root.profile.hardware_mode === "real" ? "#D8A65A" : "#69D9B1" }
                                }
                                ComboBox {
                                    id: dashboardProfiles
                                    Layout.preferredWidth: 220
                                    model: root.controller.profileNames
                                    currentIndex: Math.max(0, root.controller.profileNames.indexOf(root.profile.name))
                                    enabled: !root.controller.runActive
                                    onActivated: root.controller.selectProfile(currentText)
                                }
                                CalmButton { text: "Preflight"; enabled: !root.controller.runActive && (root.profile.execution_target !== "remote" || root.remoteState.state === "connected"); onClicked: root.controller.runPreflight() }
                                CalmButton { text: "Start run"; enabled: !root.controller.runActive && (root.profile.execution_target !== "remote" || root.controller.remoteControllable); onClicked: root.controller.startRun() }
                                CalmButton { text: "Stop all safely"; enabled: root.controller.runActive && (root.profile.execution_target !== "remote" || root.controller.remoteControllable); onClicked: root.controller.stopAll() }
                                Rectangle { width: 1; Layout.fillHeight: true; Layout.topMargin: 18; Layout.bottomMargin: 18; color: "#263746" }
                                HoldButton {
                                    text: root.profile.dual_arm ? "Hold to home both" : "Hold to home"
                                    enabled: (root.controller.state === "Ready" || root.controller.state === "Home Ready") && root.liveStatus.home_service === true && (root.liveStatus.joint_count || 0) >= (root.profile.dual_arm ? 12 : 6) && (root.profile.execution_target !== "remote" || root.controller.remoteControllable)
                                    onActivated: root.controller.requestHome()
                                }
                                HoldButton {
                                    visible: root.controller.state !== "Tracking"
                                    text: "Hold to enable tracking"
                                    enabled: (root.controller.state === "Home Ready" || root.controller.state === "Ready") && root.liveStatus.tracking_service === true && (root.profile.execution_target !== "remote" || root.controller.remoteControllable)
                                    onActivated: root.controller.enableTracking()
                                }
                                CalmButton {
                                    visible: root.controller.state === "Tracking"
                                    text: "Disable tracking"
                                    onClicked: root.controller.stopMotion()
                                }
                                SafetyButton {
                                    visible: root.controller.state === "Fault" && root.controller.runActive && root.profile.hardware_mode === "real"
                                    text: "E-stop engaged · force shutdown"
                                    onClicked: forceStopDialog.open()
                                }
                                Item { Layout.fillWidth: true }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 16
                            Panel {
                                Layout.fillWidth: true; Layout.horizontalStretchFactor: 2; Layout.preferredWidth: 400; Layout.minimumWidth: 380; Layout.preferredHeight: 660
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 16
                                    RowLayout {
                                        SectionTitle { text: "Live camera" }
                                        ViewSwitch {}
                                        Item { Layout.fillWidth: true }
                                        StatusDot { healthy: root.liveStatus.freshness !== undefined && root.liveStatus.freshness["/image"] !== undefined && root.liveStatus.freshness["/image"] < 1.0 }
                                        MutedText { text: root.liveStatus.freshness !== undefined && root.liveStatus.freshness["/image"] !== undefined && root.liveStatus.freshness["/image"] < 1.0 ? "Live" : "Waiting" }
                                    }
                                    RowLayout {
                                        Layout.fillWidth: true; Layout.fillHeight: true; spacing: 8
                                        CameraPane {
                                            Layout.fillWidth: true; Layout.fillHeight: true
                                            stream: root.cameraView === "raw" ? "raw" : "overlay"
                                        }
                                        CameraPane {
                                            visible: root.cameraView === "both"
                                            Layout.fillWidth: true; Layout.fillHeight: true
                                            stream: "raw"
                                        }
                                    }
                                }
                            }
                            Panel {
                                Layout.fillWidth: true; Layout.horizontalStretchFactor: 3; Layout.preferredWidth: 600; Layout.minimumWidth: 480; Layout.preferredHeight: 660
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 16
                                    RowLayout {
                                        Layout.fillWidth: true; spacing: 10
                                        SectionTitle { text: "3D view" }
                                        Repeater {
                                            // One toggle per running source; colours match the arrows.
                                            model: [["yolo", root.visualName, "#338CFF", root.profile.yolo_enabled],
                                                    ["mobilenetv2", "MobileNetV2", "#F21AD9", root.profile.mobilenetv2_enabled],
                                                    ["ast", "AST", "#EBEBEB", root.profile.ast_enabled],
                                                    ["gre", "GRE", "#1AF2F2", root.profile.gre_enabled],
                                                    ["ekf", "EKF", "#FFB30D", root.profile.yolo_enabled || root.profile.mobilenetv2_enabled]]
                                                .filter(arrow => arrow[3])
                                            delegate: CheckBox {
                                                id: arrowToggle
                                                required property var modelData
                                                text: modelData[1]
                                                checked: JSON.parse(root.controller.arrowsJson)[modelData[0]] !== false
                                                onToggled: root.controller.setArrowVisible(modelData[0], checked)
                                                indicator: Rectangle {
                                                    x: arrowToggle.leftPadding; anchors.verticalCenter: parent.verticalCenter
                                                    width: 16; height: 16; radius: 4
                                                    color: arrowToggle.checked ? arrowToggle.modelData[2] : "transparent"
                                                    border.color: arrowToggle.modelData[2]; border.width: 2
                                                }
                                            }
                                        }
                                        Item { Layout.fillWidth: true }
                                        Switch {
                                            text: "Show"
                                            checked: root.profile.use_rviz
                                            onToggled: root.controller.setThreeDView(checked)
                                        }
                                    }
                                    Rectangle {
                                        Layout.fillWidth: true; Layout.fillHeight: true
                                        radius: 10; color: "#080D12"; clip: true
                                        // RViz's own window is placed over this area (see x11_embed.py).
                                        Item {
                                            id: rvizArea
                                            anchors.fill: parent; anchors.margins: 2
                                            Timer {
                                                interval: 100; repeat: true
                                                running: root.controller.rvizEmbedded
                                                onTriggered: rvizArea.report()
                                            }
                                            function report() {
                                                let topLeft = rvizArea.mapToItem(null, 0, 0)
                                                let view = dashboardScroll.mapToItem(null, 0, 0)
                                                // Only the part inside the scrolling page is visible.
                                                let top = Math.max(topLeft.y, view.y)
                                                let bottom = Math.min(topLeft.y + rvizArea.height, view.y + dashboardScroll.height)
                                                let covered = preflightDrawer.visible || closeDialog.visible || forceStopDialog.visible
                                                    || dashboardProfiles.popup.visible
                                                let shown = root.pageNames[root.currentPage] === "Dashboard" && !covered && bottom - top > 40
                                                root.controller.setRvizArea(topLeft.x, top, rvizArea.width, Math.max(0, bottom - top), shown)
                                            }
                                        }
                                        MutedText {
                                            anchors.centerIn: parent
                                            visible: !root.controller.rvizEmbedded
                                            text: root.controller.rvizStatus
                                        }
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
                                    MutedText { text: root.profile.dual_arm ? "Dual-cobot joint feedback" : "Joint feedback" }
                                    Label { text: (root.liveStatus.joint_count || 0) + " joints"; color: "#DDEAF1"; font.pixelSize: 18 }
                                    MutedText {
                                        visible: root.profile.dual_arm
                                        property var arms: root.liveStatus.arms || ({})
                                        text: "Left " + ((arms.left && arms.left.joint_count) || 0) + "/6 · Right " + ((arms.right && arms.right.joint_count) || 0) + "/6"
                                        color: root.liveStatus.dual_arm_detected ? "#69D9B1" : "#D8A65A"
                                    }
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
                                    MutedText { text: "EKF preferred, then " + root.visualName + " or MobileNetV2" }
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
                                    MutedText { text: "Audio: " + (root.liveStatus.freshness && root.liveStatus.freshness["/sst"] < 1.0 ? "fresh" : "waiting") + "   ·   Detections: " + (root.liveStatus.freshness && root.liveStatus.freshness["/detections_output"] < 1.0 ? "fresh" : "waiting") }
                                }
                            }
                        }
                    }
                }

                // Perception
                ScrollView {
                    id: perceptionScroll
                    contentWidth: availableWidth
                    property var t: root.telemetry
                    property var yolo: t.yolo || ({})
                    property var mnet: t.mobilenetv2 || ({})
                    property var ekf: t.ekf || ({})
                    property var dirs: t.directions || ({})
                    property var cam: t.camera || ({})
                    property var ee: t.ee || ({})
                    property bool live: t.t !== undefined && (Date.now() / 1000 - t.t) < 2.0
                    ColumnLayout {
                        x: 22; width: perceptionScroll.availableWidth - 44; spacing: 14
                        Item { Layout.preferredHeight: 6 }
                        RowLayout {
                            Layout.fillWidth: true
                            SectionTitle { text: "Perception & monitoring" }
                            StatusDot { healthy: perceptionScroll.live }
                            MutedText { text: perceptionScroll.live ? "Telemetry live (10 Hz)" : "Waiting for /dracoviloc/telemetry: start a run" }
                            Item { Layout.fillWidth: true }
                            ViewSwitch {}
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 14
                            Panel {
                                Layout.fillWidth: true; Layout.preferredWidth: 3; Layout.preferredHeight: 470
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 14; spacing: 8
                                    RowLayout {
                                        Layout.fillWidth: true
                                        SectionTitle { text: "Camera" }
                                        Item { Layout.fillWidth: true }
                                        MutedText { text: root.fmt(perceptionScroll.cam.fps, 1) + " fps · " + (perceptionScroll.cam.width || "–") + "×" + (perceptionScroll.cam.height || "–") + " · raw recorded always" }
                                    }
                                    RowLayout {
                                        Layout.fillWidth: true; Layout.fillHeight: true; spacing: 8
                                        CameraPane { Layout.fillWidth: true; Layout.fillHeight: true; stream: root.cameraView === "overlay" ? "overlay" : "raw" }
                                        CameraPane { visible: root.cameraView === "both"; Layout.fillWidth: true; Layout.fillHeight: true; stream: "overlay" }
                                    }
                                }
                            }
                            Panel {
                                Layout.fillWidth: true; Layout.preferredWidth: 2; Layout.preferredHeight: 470
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 14; spacing: 10
                                    SectionTitle { text: root.visualName }
                                    GridLayout {
                                        columns: 3; columnSpacing: 18; rowSpacing: 8
                                        Metric { label: "Inference rate"; value: root.fmt(perceptionScroll.yolo.inference_rate_hz, 1) + " Hz" }
                                        Metric { label: "Detections"; value: (perceptionScroll.yolo.detections === undefined ? "–" : perceptionScroll.yolo.detections) }
                                        Metric { label: "Best score"; value: root.fmt(perceptionScroll.yolo.best_score, 2); valueColor: (perceptionScroll.yolo.best_score || 0) > 0.5 ? "#5BD08A" : "#EEF5F8" }
                                        Metric { label: "Latency"; value: root.fmt(perceptionScroll.yolo.latency_ms, 0) + " ms" }
                                        Metric { label: "Azimuth"; value: root.fmt(perceptionScroll.dirs.yolo ? perceptionScroll.dirs.yolo.azimuth_deg : NaN, 1) + "°" }
                                        Metric { label: "Elevation"; value: root.fmt(perceptionScroll.dirs.yolo ? perceptionScroll.dirs.yolo.elevation_deg : NaN, 1) + "°" }
                                    }
                                    MutedText { text: "Best score (15 s)" }
                                    Sparkline { Layout.fillWidth: true; values: root.history.score; lineColor: "#5BD08A" }
                                    MutedText { text: "Camera → detection latency, ms (15 s)" }
                                    Sparkline { Layout.fillWidth: true; values: root.history.latency; lineColor: "#E8C468" }
                                    MutedText { Layout.fillWidth: true; wrapMode: Text.Wrap; text: "Directions are unit vectors in " + ((perceptionScroll.dirs.yolo && perceptionScroll.dirs.yolo.frame) || "the tracking frame") + "; azimuth = atan2(y, x), elevation = asin(z)." }
                                    Item { Layout.fillHeight: true }
                                }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 14
                            Panel {
                                Layout.fillWidth: true; Layout.preferredHeight: 330
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 14; spacing: 8
                                    RowLayout {
                                        SectionTitle { text: "MobileNetV2" }
                                        Item { Layout.fillWidth: true }
                                        MutedText { text: root.fmt(perceptionScroll.mnet.window_rate_hz, 1) + " windows/s · " + root.fmt(perceptionScroll.mnet.inference_ms, 1) + " ms" }
                                    }
                                    Repeater {
                                        model: perceptionScroll.mnet.channels || []
                                        delegate: RowLayout {
                                            id: channelRow
                                            required property var modelData
                                            Layout.fillWidth: true; spacing: 8
                                            opacity: modelData.age_s !== null && modelData.age_s < 3 ? 1.0 : 0.45
                                            Label { text: "ch " + channelRow.modelData.channel; color: "#93A7B8"; font.family: "monospace"; Layout.preferredWidth: 40 }
                                            Rectangle {
                                                Layout.fillWidth: true; height: 16; radius: 4; color: "#0E151D"; border.color: "#1E2C39"
                                                Rectangle {
                                                    width: parent.width * Math.max(0, Math.min(1, channelRow.modelData.probability || 0)); height: parent.height; radius: 4
                                                    color: channelRow.modelData.decision ? "#5BD08A" : "#35C6D0"
                                                }
                                                Rectangle { x: parent.width * 0.75; width: 2; height: parent.height; color: "#E8C468"; opacity: 0.8 }
                                            }
                                            Label { text: root.fmt(channelRow.modelData.probability, 2); color: "#D9E6EE"; font.family: "monospace"; Layout.preferredWidth: 40 }
                                            Label { text: channelRow.modelData.votes + "/" + channelRow.modelData.vote_window; color: "#93A7B8"; font.family: "monospace"; Layout.preferredWidth: 32 }
                                            Label { text: channelRow.modelData.decision ? "DRONE" : (channelRow.modelData.actionable ? "–" : "diag"); color: channelRow.modelData.decision ? "#5BD08A" : "#7E8F9C"; font.bold: true; Layout.preferredWidth: 52 }
                                            Label { text: "track " + channelRow.modelData.track_id; color: "#7E8F9C"; Layout.preferredWidth: 60 }
                                        }
                                    }
                                    MutedText { visible: !(perceptionScroll.mnet.channels || []).length; text: "No inference windows yet (needs active ODAS tracks)." }
                                    MutedText { Layout.fillWidth: true; wrapMode: Text.Wrap; text: "Bar = drone probability; yellow tick = decision threshold. Decisions and published directions are unchanged; this view only reports them." }
                                    Item { Layout.fillHeight: true }
                                }
                            }
                            Panel {
                                Layout.fillWidth: true; Layout.preferredHeight: 330
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 14; spacing: 8
                                    RowLayout {
                                        SectionTitle { text: "EKF fusion" }
                                        Item { Layout.fillWidth: true }
                                        MutedText { text: "fused az " + root.fmt(perceptionScroll.dirs.ekf ? perceptionScroll.dirs.ekf.azimuth_deg : NaN, 1) + "° · el " + root.fmt(perceptionScroll.dirs.ekf ? perceptionScroll.dirs.ekf.elevation_deg : NaN, 1) + "°" }
                                    }
                                    GridLayout {
                                        Layout.fillWidth: true; columns: 6; columnSpacing: 12; rowSpacing: 4
                                        Repeater {
                                            model: ["Source", "Rate", "Accepted", "Rejected", "Last NIS", "Innovation"]
                                            delegate: Label { required property string modelData; text: modelData; color: "#7E8F9C"; font.pixelSize: 11 }
                                        }
                                        Repeater {
                                            model: Object.keys(perceptionScroll.ekf.sources || {})
                                            delegate: Repeater {
                                                id: sourceRow
                                                required property string modelData
                                                property var src: perceptionScroll.ekf.sources[modelData]
                                                property var counts: src ? src.decisions : ({})
                                                property int accepted: (counts.accepted || 0) + (counts.initialized || 0) + (counts.reacquired || 0)
                                                property int rejected: (counts.rejected_nis || 0) + (counts.rejected_bearing_gate || 0) + (counts.rejected_first_arrival || 0)
                                                model: [
                                                    modelData.replace("/direction", "").replace("/", ""),
                                                    root.fmt(src ? src.rate_hz : NaN, 1) + " Hz",
                                                    String(accepted),
                                                    String(rejected) + (rejected ? " (" + ((counts.rejected_nis || 0) ? "NIS " + counts.rejected_nis : "") + ((counts.rejected_bearing_gate || 0) ? " gate " + counts.rejected_bearing_gate : "") + ")" : ""),
                                                    root.fmt(src ? src.last_nis : NaN, 2),
                                                    root.fmt(src ? src.last_innovation_deg : NaN, 1) + "°"
                                                ]
                                                delegate: Label { required property string modelData; text: modelData; color: "#D9E6EE"; font.family: "monospace" }
                                            }
                                        }
                                    }
                                    MutedText { text: "NIS of the latest update (15 s) · gate " + root.fmt(perceptionScroll.ekf.nis_gate, 2) }
                                    Sparkline { Layout.fillWidth: true; values: root.history.nis; threshold: perceptionScroll.ekf.nis_gate === undefined || perceptionScroll.ekf.nis_gate === null ? NaN : perceptionScroll.ekf.nis_gate; lineColor: "#93BFFF" }
                                    Item { Layout.fillHeight: true }
                                }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 14
                            Panel {
                                Layout.fillWidth: true; Layout.preferredWidth: 3; Layout.preferredHeight: 400
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 14; spacing: 6
                                    RowLayout {
                                        SectionTitle { text: "Arms" }
                                        Item { Layout.fillWidth: true }
                                        MutedText { text: "joint states " + root.fmt(perceptionScroll.t.joints ? perceptionScroll.t.joints.rate_hz : NaN, 0) + " Hz · rad, rad/s, rad/s², rad/s³" }
                                    }
                                    GridLayout {
                                        Layout.fillWidth: true; columns: 5; columnSpacing: 16; rowSpacing: 2
                                        Repeater {
                                            model: ["Joint", "Position", "Velocity", "Acceleration", "Jerk"]
                                            delegate: Label { required property string modelData; text: modelData; color: "#7E8F9C"; font.pixelSize: 11 }
                                        }
                                        Repeater {
                                            model: perceptionScroll.t.joints ? perceptionScroll.t.joints.items : []
                                            delegate: Repeater {
                                                required property var modelData
                                                model: [modelData.name, root.fmt(modelData.position, 3), root.fmt(modelData.velocity, 3), root.fmt(modelData.acceleration, 2), root.fmt(modelData.jerk, 1)]
                                                delegate: Label { required property string modelData; text: modelData; color: "#D9E6EE"; font.family: "monospace"; font.pixelSize: 12 }
                                            }
                                        }
                                    }
                                    Item { Layout.fillHeight: true }
                                }
                            }
                            Panel {
                                Layout.fillWidth: true; Layout.preferredWidth: 2; Layout.preferredHeight: 400
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 14; spacing: 8
                                    SectionTitle { text: "End effector & audio" }
                                    RowLayout {
                                        spacing: 18
                                        Metric { label: "Speed m/s"; value: root.fmt(perceptionScroll.ee.speed, 3) }
                                        Metric { label: "Accel m/s²"; value: root.fmt(perceptionScroll.ee.acceleration, 2) }
                                        Metric { label: "Jerk m/s³"; value: root.fmt(perceptionScroll.ee.jerk, 1) }
                                    }
                                    MutedText { text: "End-effector speed (15 s)" }
                                    Sparkline { Layout.fillWidth: true; values: root.history.ee; lineColor: "#D78BF0" }
                                    RowLayout {
                                        Layout.fillWidth: true; Layout.fillHeight: true; spacing: 12
                                        Canvas {
                                            id: polar
                                            Layout.preferredWidth: 150; Layout.preferredHeight: 150
                                            property var sources: perceptionScroll.t.audio ? perceptionScroll.t.audio.sources : []
                                            onSourcesChanged: requestPaint()
                                            onPaint: {
                                                let ctx = getContext("2d"); ctx.reset()
                                                let cx = width / 2, cy = height / 2, r = Math.min(cx, cy) - 6
                                                ctx.strokeStyle = "#263746"; ctx.lineWidth = 1
                                                for (let k = 1; k <= 3; ++k) { ctx.beginPath(); ctx.arc(cx, cy, r * k / 3, 0, 2 * Math.PI); ctx.stroke() }
                                                ctx.beginPath(); ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy); ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r); ctx.stroke()
                                                for (let s of sources) {
                                                    // radius shrinks with elevation: horizon on the rim, zenith at the centre
                                                    let rr = r * (1 - Math.max(0, s.elevation_deg) / 90)
                                                    let a = s.azimuth_deg * Math.PI / 180
                                                    ctx.fillStyle = Qt.rgba(0.34, 0.79, 0.83, 0.35 + 0.65 * Math.min(1, s.activity))
                                                    ctx.beginPath(); ctx.arc(cx + rr * Math.cos(a), cy - rr * Math.sin(a), 4 + 6 * Math.min(1, s.activity), 0, 2 * Math.PI); ctx.fill()
                                                }
                                            }
                                        }
                                        ColumnLayout {
                                            Layout.fillWidth: true
                                            MutedText { text: "ODAS tracked sources" }
                                            MutedText { text: root.fmt(perceptionScroll.t.audio ? perceptionScroll.t.audio.rate_hz : NaN, 0) + " Hz · " + (polar.sources || []).length + " active" }
                                            MutedText { Layout.fillWidth: true; wrapMode: Text.Wrap; text: "Top view: azimuth around, elevation toward the centre; dot size = activity." }
                                        }
                                    }
                                }
                            }
                        }
                        Item { Layout.preferredHeight: 10 }
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
                        Panel {
                            Layout.fillWidth: true
                            Layout.preferredHeight: root.profile.execution_target === "remote" ? 320 : 128
                            GridLayout {
                                anchors.fill: parent; anchors.margins: 18
                                columns: 4; columnSpacing: 14; rowSpacing: 10
                                SectionTitle { text: "Execution target"; Layout.columnSpan: 4 }
                                MutedText { text: "Run on" }
                                ComboBox {
                                    model: ["local", "remote"]
                                    currentIndex: root.profile.execution_target === "remote" ? 1 : 0
                                    enabled: !root.controller.runActive
                                    onActivated: root.controller.setOption("execution_target", currentText)
                                }
                                MutedText {
                                    text: root.profile.execution_target === "remote" ? "Tracking continues if this laptop disconnects." : "Processes run on this machine."
                                    Layout.columnSpan: 2
                                    color: root.profile.execution_target === "remote" ? "#D8A65A" : "#8FA5B8"
                                }
                                MutedText { visible: root.profile.execution_target === "remote"; text: "SSH alias" }
                                TextField {
                                    visible: root.profile.execution_target === "remote"
                                    text: root.profile.ssh_alias || "jetson"
                                    enabled: !root.controller.runActive
                                    onEditingFinished: root.controller.setOption("ssh_alias", text)
                                }
                                MutedText { visible: root.profile.execution_target === "remote"; text: "Project on Jetson" }
                                TextField {
                                    visible: root.profile.execution_target === "remote"
                                    text: root.profile.jetson_project_root || "~/DracoViLoc"
                                    enabled: !root.controller.runActive
                                    onEditingFinished: root.controller.setOption("jetson_project_root", text)
                                }
                                MutedText { visible: root.profile.execution_target === "remote"; text: "Isaac workspace" }
                                TextField {
                                    visible: root.profile.execution_target === "remote"
                                    text: root.profile.jetson_isaac_root || "~/workspaces/isaac_ros-dev"
                                    enabled: !root.controller.runActive
                                    onEditingFinished: root.controller.setOption("jetson_isaac_root", text)
                                }
                                MutedText { visible: root.profile.execution_target === "remote"; text: "Recordings on Jetson" }
                                TextField {
                                    visible: root.profile.execution_target === "remote"
                                    text: root.profile.remote_recordings_root || "~/DracoViLoc/runs"
                                    enabled: !root.controller.runActive
                                    onEditingFinished: root.controller.setOption("remote_recordings_root", text)
                                }
                                MutedText {
                                    visible: root.profile.execution_target === "remote"
                                    text: "Status: " + (root.remoteState.state || "disconnected") + (root.remoteState.has_control ? " · controller" : " · read-only")
                                    Layout.columnSpan: 2
                                }
                                RowLayout {
                                    visible: root.profile.execution_target === "remote"
                                    Layout.columnSpan: 2
                                    CalmButton {
                                        text: root.remoteState.state === "connected" ? "Disconnect" : "Connect"
                                        onClicked: root.remoteState.state === "connected" ? root.controller.disconnectRemote() : root.controller.connectRemote()
                                    }
                                    CalmButton {
                                        text: "Take control"
                                        enabled: root.remoteState.state === "connected" && !root.remoteState.has_control
                                        onClicked: root.controller.acquireRemoteControl()
                                    }
                                    CalmButton {
                                        text: "Deploy / update agent"
                                        enabled: !root.controller.runActive
                                        onClicked: root.controller.deployRemoteAgent()
                                    }
                                }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 16
                            Panel {
                                Layout.fillWidth: true; Layout.preferredHeight: 590
                                ColumnLayout {
                                    anchors.fill: parent; anchors.margins: 18; spacing: 10
                                    SectionTitle { text: "Pipeline" }

                                    Label { text: "Audio"; color: "#67DFE7"; font.bold: true }
                                    RowLayout {
                                        id: audioChoice
                                        spacing: 4
                                        property string current: root.profile.audio_enabled ? "odas" : "off"
                                        Repeater {
                                            model: [["off", "Off"], ["odas", "ODAS (UMA16)"]]
                                            delegate: RadioButton {
                                                required property var modelData
                                                text: modelData[1]
                                                checked: audioChoice.current === modelData[0]
                                                enabled: !root.controller.runActive
                                                onClicked: root.controller.setOption(
                                                    "audio_enabled", modelData[0] === "odas")
                                            }
                                        }
                                    }
                                    RowLayout {
                                        spacing: 18
                                        CheckBox { text: "MobileNetV2"; checked: root.profile.mobilenetv2_enabled; enabled: !root.controller.runActive && root.profile.audio_enabled; onToggled: root.controller.setOption("mobilenetv2_enabled", checked) }
                                        CheckBox { text: "AST"; checked: root.profile.ast_enabled; enabled: !root.controller.runActive && root.profile.audio_enabled; onToggled: root.controller.setOption("ast_enabled", checked) }
                                        CheckBox { text: "GRE"; checked: root.profile.gre_enabled; enabled: !root.controller.runActive && root.profile.audio_enabled; onToggled: root.controller.setOption("gre_enabled", checked) }
                                    }
                                    RowLayout {
                                        Layout.fillWidth: true
                                        visible: root.profile.audio_enabled
                                        spacing: 12
                                        ColumnLayout {
                                            Layout.fillWidth: true
                                            spacing: 2
                                            MutedText { text: "Minimum sound activity" }
                                            MutedText {
                                                Layout.fillWidth: true
                                                text: "Lower accepts quieter sounds; higher rejects more background noise."
                                                wrapMode: Text.Wrap
                                            }
                                        }
                                        SpinBox {
                                            id: minimumSoundActivity
                                            from: 0
                                            to: 100
                                            stepSize: 1
                                            value: Math.round(Number(root.profile.min_activity) * 100)
                                            editable: true
                                            enabled: !root.controller.runActive
                                            Layout.preferredWidth: 110
                                            textFromValue: function(value, locale) {
                                                return Number(value / 100).toLocaleString(locale, "f", 2)
                                            }
                                            valueFromText: function(text, locale) {
                                                return Math.round(Number.fromLocaleString(locale, text) * 100)
                                            }
                                            onValueModified: root.controller.setOption("min_activity", value / 100.0)
                                        }
                                    }

                                    Rectangle { Layout.fillWidth: true; height: 1; color: "#263746" }
                                    Label { text: "Visual"; color: "#67DFE7"; font.bold: true }
                                    RowLayout {
                                        id: detectorChoice
                                        spacing: 4
                                        // One visual detector at a time; all feed the same EKF input.
                                        property string current: root.profile.yolo_enabled ? (root.profile.visual_detector || "yolo") : "off"
                                        Repeater {
                                            model: [["off", "Off"], ["yolo", "YOLO"], ["dfine", "D-FINE"], ["dfine_n", "D-FINE-N"]]
                                            delegate: RadioButton {
                                                required property var modelData
                                                text: modelData[1]
                                                checked: detectorChoice.current === modelData[0]
                                                enabled: !root.controller.runActive
                                                onClicked: {
                                                    if (modelData[0] === "off") {
                                                        root.controller.setOption("yolo_enabled", false)
                                                    } else {
                                                        root.controller.setOption("visual_detector", modelData[0])
                                                        root.controller.setOption("yolo_enabled", true)
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    ComboBox {
                                        id: visualModelChoice
                                        Layout.fillWidth: true
                                        visible: root.profile.yolo_enabled
                                        enabled: !root.controller.runActive
                                        property string detector: root.profile.visual_detector || "yolo"
                                        property string option: ({ yolo: "yolo_model", dfine: "dfine_model", dfine_n: "dfine_n_model" })[detector]
                                        property var mine: (root.setup.models || []).filter(m => m.detector === detector && !m.legacy)
                                        property var ready: mine.filter(m => m.ready)
                                        property var autoModel: ready.find(m => m.id === (root.setup.autos || {})[detector])
                                        property var ids: ["auto"].concat(ready.map(m => m.id))
                                        // Dates only; a name is added only when two models share a date.
                                        function label(m) {
                                            return ready.filter(o => o.date === m.date).length > 1 ? m.date + " · " + m.experiment : m.date
                                        }
                                        model: ["Newest" + (autoModel ? " (" + label(autoModel) + ")" : "")].concat(ready.map(m => label(m)))
                                        currentIndex: Math.max(0, ids.indexOf(root.profile[option] || "auto"))
                                        onActivated: (index) => root.controller.setOption(option, ids[index])
                                    }
                                    RowLayout {
                                        Layout.fillWidth: true
                                        visible: root.profile.yolo_enabled &&
                                                 (root.profile.visual_detector || "yolo") === "yolo"
                                        spacing: 12
                                        ColumnLayout {
                                            Layout.fillWidth: true
                                            spacing: 2
                                            MutedText { text: "YOLO confidence threshold" }
                                            MutedText {
                                                Layout.fillWidth: true
                                                text: "Higher rejects weak detections; lower accepts more detections."
                                                wrapMode: Text.Wrap
                                            }
                                        }
                                        SpinBox {
                                            id: yoloConfidenceThreshold
                                            from: 0
                                            to: 100
                                            stepSize: 1
                                            value: Math.round(Number(root.profile.yolo_confidence_threshold) * 100)
                                            editable: true
                                            enabled: !root.controller.runActive
                                            Layout.preferredWidth: 110
                                            textFromValue: function(value, locale) {
                                                return Number(value / 100).toLocaleString(locale, "f", 2)
                                            }
                                            valueFromText: function(text, locale) {
                                                return Math.round(Number.fromLocaleString(locale, text) * 100)
                                            }
                                            onValueModified: root.controller.setOption(
                                                "yolo_confidence_threshold", value / 100.0)
                                        }
                                    }
                                    MutedText {
                                        // Newer models of this detector that cannot run yet, and why.
                                        property var pending: visualModelChoice.mine.filter(m => !m.ready && (!visualModelChoice.autoModel || m.date >= visualModelChoice.autoModel.date))
                                        visible: root.profile.yolo_enabled && (pending.length > 0 || visualModelChoice.ready.length === 0)
                                        Layout.fillWidth: true; wrapMode: Text.Wrap
                                        color: "#E8C468"
                                        text: {
                                            if (pending.length === 0)
                                                return "No " + root.visualName + " model yet: fetch one in Setup & builds."
                                            return pending.map(m => {
                                                const building = (root.setup.current_title || "").indexOf(m.id) >= 0
                                                return m.date + (building ? ": building its TensorRT plan…"
                                                    : (m.onnx_state !== "ok" ? ": needs an ONNX export" : ": needs a TensorRT plan")) 
                                            }).join("\n") + "\n(Setup & builds › Visual models; it appears here when ready.)"
                                        }
                                    }

                                    Rectangle { Layout.fillWidth: true; height: 1; color: "#263746" }
                                    Label { text: "Fusion"; color: "#67DFE7"; font.bold: true }
                                    MutedText {
                                        Layout.fillWidth: true; wrapMode: Text.Wrap
                                        property var inputs: [root.profile.yolo_enabled ? root.visualName : "", root.profile.mobilenetv2_enabled ? "MobileNetV2" : ""].filter(x => x)
                                        text: inputs.length ? "EKF fuses " + inputs.join(" + ") + "." : "EKF has no input: choose a visual detector or turn on MobileNetV2."
                                        color: inputs.length ? "#C8D7E0" : "#E8C468"
                                    }
                                    MutedText {
                                        text: root.profile.audio_enabled && root.profile.yolo_enabled
                                            ? "Raw camera and audio are recorded."
                                            : (root.profile.audio_enabled ? "Audio is recorded."
                                               : (root.profile.yolo_enabled ? "Raw camera is recorded."
                                                  : "No sensor recording is enabled."))
                                    }
                                    Item { Layout.fillHeight: true }
                                }
                            }
                            Panel {
                                Layout.fillWidth: true; Layout.preferredHeight: 590
                                GridLayout {
                                    anchors.fill: parent; anchors.margins: 18; columns: 2; columnSpacing: 18; rowSpacing: 12
                                    SectionTitle { text: "Robot and tracking"; Layout.columnSpan: 2 }
                                    MutedText { text: "Arms" }
                                    ComboBox {
                                        Layout.fillWidth: true
                                        model: ["Mock", "Real FAIRINO cobots"]
                                        currentIndex: root.profile.hardware_mode === "real" ? 1 : 0
                                        enabled: !root.controller.runActive
                                        onActivated: (index) => root.controller.setOption("hardware_mode", index === 1 ? "real" : "mock")
                                    }
                                    MutedText { text: "Cobot configuration" }
                                    Switch { text: "Two cobots"; checked: root.profile.dual_arm; enabled: !root.controller.runActive; onToggled: root.controller.setOption("dual_arm", checked) }
                                    MutedText { visible: root.profile.hardware_mode === "real"; text: root.profile.dual_arm ? "Left cobot IP" : "Cobot IP" }
                                    TextField { visible: root.profile.hardware_mode === "real"; text: root.profile.dual_arm ? root.profile.left_robot_ip : root.profile.robot_ip; enabled: !root.controller.runActive; onEditingFinished: root.controller.setOption(root.profile.dual_arm ? "left_robot_ip" : "robot_ip", text) }
                                    MutedText { visible: root.profile.hardware_mode === "real" && root.profile.dual_arm; text: "Right cobot IP" }
                                    TextField { visible: root.profile.hardware_mode === "real" && root.profile.dual_arm; text: root.profile.right_robot_ip; enabled: !root.controller.runActive; onEditingFinished: root.controller.setOption("right_robot_ip", text) }
                                    CalmButton { visible: root.profile.hardware_mode === "real"; text: "Check connection"; onClicked: root.controller.checkRobots() }
                                    ColumnLayout {
                                        visible: root.profile.hardware_mode === "real"
                                        Layout.fillWidth: true; spacing: 2
                                        Repeater {
                                            model: Object.keys(root.robotCheck)
                                            delegate: Label {
                                                required property string modelData
                                                property var result: root.robotCheck[modelData]
                                                Layout.fillWidth: true; wrapMode: Text.Wrap; font.pixelSize: 12
                                                text: result.ip + ": " + (result.state === "checking" ? "checking…" : result.detail)
                                                color: result.state === "ok" ? "#5BD08A" : (result.state === "checking" ? "#93A7B8" : (result.state === "warning" ? "#E8C468" : "#FF8992"))
                                            }
                                        }
                                        MutedText { visible: Object.keys(root.robotCheck).length === 0; text: "Ping + FAIRINO port 20003. Safe while running." }
                                    }
                                    MutedText { text: "Arms follow" }
                                    ComboBox {
                                        id: modeBox
                                        Layout.fillWidth: true
                                        enabled: !root.controller.runActive
                                        property var choices: [["off", "Nothing (observe only)"]]
                                            .concat(root.profile.yolo_enabled || root.profile.mobilenetv2_enabled ? [["ekf", "Fused direction (EKF)"]] : [])
                                            .concat(root.profile.yolo_enabled ? [["direct_yolo", root.visualName + " only"]] : [])
                                            .concat(root.profile.mobilenetv2_enabled ? [["direct_mobilenetv2", "MobileNetV2 only"]] : [])
                                            .concat(root.profile.ast_enabled ? [["direct_ast", "AST only"]] : [])
                                            .concat(root.profile.gre_enabled ? [["direct_gre", "GRE only"]] : [])
                                            .concat(root.profile.ast_enabled && root.profile.gre_enabled ? [["direct_either", "AST or GRE"]] : [])
                                        model: choices.map(choice => choice[1])
                                        currentIndex: Math.max(0, choices.findIndex(choice => choice[0] === root.profile.tracking_mode))
                                        onActivated: (index) => root.controller.setOption("tracking_mode", choices[index][0])
                                    }
                                    MutedText { text: "Use RViz" }
                                    Switch { checked: root.profile.use_rviz; onToggled: root.controller.setOption("use_rviz", checked) }
                                    MutedText { text: "MoveIt" }
                                    Switch { checked: root.profile.use_moveit; onToggled: root.controller.setOption("use_moveit", checked) }
                                }
                            }
                        }
                        Panel {
                            visible: root.profile.dual_arm
                            Layout.fillWidth: true; Layout.preferredHeight: 235
                            GridLayout {
                                anchors.fill: parent; anchors.margins: 18
                                columns: 4; columnSpacing: 18; rowSpacing: 10
                                SectionTitle { text: "Dual-cobot geometry and coordination"; Layout.columnSpan: 4 }
                                MutedText { text: "Right cobot X (m)" }
                                TextField { text: root.profile.second_arm_x; enabled: !root.controller.runActive; onEditingFinished: root.controller.setOption("second_arm_x", text) }
                                MutedText { text: "Right cobot Y (m)" }
                                TextField { text: root.profile.second_arm_y; enabled: !root.controller.runActive; onEditingFinished: root.controller.setOption("second_arm_y", text) }
                                MutedText { text: "Right cobot Z (m)" }
                                TextField { text: root.profile.second_arm_z; enabled: !root.controller.runActive; onEditingFinished: root.controller.setOption("second_arm_z", text) }
                                MutedText { text: "Tracking policy" }
                                ComboBox {
                                    model: ["first_arm_lead", "simultaneous"]
                                    currentIndex: model.indexOf(root.profile.dual_tracking_policy)
                                    enabled: !root.controller.runActive
                                    onActivated: root.controller.setOption("dual_tracking_policy", currentText)
                                }
                                MutedText { text: "Lead tolerance (rad)" }
                                TextField { text: root.profile.lead_lock_tolerance_rad; enabled: !root.controller.runActive; onEditingFinished: root.controller.setOption("lead_lock_tolerance_rad", text) }
                                MutedText { text: "first_arm_lead waits for the sensor-side cobot to align before moving the follower; simultaneous moves both together."; Layout.columnSpan: 4 }
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
                            id: advancedPanel
                            property bool open: false
                            Layout.fillWidth: true
                            Layout.preferredHeight: advancedColumn.implicitHeight + 36
                            ColumnLayout {
                                id: advancedColumn
                                anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 18
                                spacing: 10
                                RowLayout {
                                    Layout.fillWidth: true
                                    ColumnLayout {
                                        Layout.fillWidth: true; spacing: 2
                                        SectionTitle { text: "Advanced launch settings" }
                                        MutedText {
                                            Layout.fillWidth: true; wrapMode: Text.Wrap
                                            text: Object.keys(root.profile.expert_arguments || {}).length
                                                  ? Object.keys(root.profile.expert_arguments).length + " changed from the defaults (saved with the profile)."
                                                  : "Every launch setting not shown above (EKF gates, microphone angles, ...). Nothing changed."
                                        }
                                    }
                                    CalmButton { text: advancedPanel.open ? "Hide" : "Show"; onClicked: advancedPanel.open = !advancedPanel.open }
                                }
                                Repeater {
                                    model: Object.keys(root.profile.expert_arguments || {})
                                    delegate: RowLayout {
                                        required property string modelData
                                        Layout.fillWidth: true; spacing: 10
                                        Label { text: modelData + " = " + root.profile.expert_arguments[modelData]; color: "#D9E6EE"; font.family: "monospace"; Layout.fillWidth: true }
                                        CalmButton { text: "Remove"; implicitHeight: 32; enabled: !root.controller.runActive; onClicked: root.controller.removeExpertOption(modelData) }
                                    }
                                }
                                RowLayout {
                                    visible: advancedPanel.open
                                    Layout.fillWidth: true; spacing: 12
                                    ComboBox {
                                        id: expertName
                                        Layout.preferredWidth: 320
                                        editable: true
                                        model: root.controller.launchArgumentNames
                                    }
                                    TextField { id: expertValue; Layout.fillWidth: true; placeholderText: "New value" }
                                    CalmButton {
                                        text: "Set"
                                        enabled: !root.controller.runActive && expertName.editText.length > 0 && expertValue.text.length > 0
                                        onClicked: root.controller.setExpertOption(expertName.editText, expertValue.text)
                                    }
                                }
                            }
                        }
                    }
                }

                // Recordings
                Item {
                    ColumnLayout {
                        anchors.fill: parent; spacing: 14; anchors.margins: 22
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
                                        CalmButton { text: root.profile.execution_target === "remote" ? "Download" : "Open"; enabled: root.profile.execution_target !== "remote" || state === "complete"; onClicked: root.controller.openPath(path) }
                                    }
                                }
                            }
                        }
                    }
                }

                // Logs
                Item {
                    ColumnLayout {
                        anchors.fill: parent; spacing: 12; anchors.margins: 22
                        RowLayout {
                            Layout.fillWidth: true; spacing: 10
                            SectionTitle { text: "Logs" }
                            MutedText {
                                Layout.fillWidth: true; elide: Text.ElideMiddle
                                text: root.controller.runDirectory ? "Current run: " + root.controller.runDirectory : "Outside runs: " + root.controller.sessionLogDirectory
                            }
                            CalmButton { text: root.controller.runDirectory ? "Open run folder" : "Open logs folder"; onClicked: root.controller.openSessionFolder() }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 6
                            Repeater {
                                model: [{"name": "", "state": "events"}].concat(root.processes)
                                delegate: Button {
                                    id: tabButton
                                    required property var modelData
                                    property bool selected: root.selectedTerminal === modelData.name
                                    flat: true; implicitHeight: 34
                                    text: modelData.name === "" ? "Events" : modelData.name
                                    onClicked: {
                                        root.controller.uiEvent("open_log_tab", text)
                                        root.selectedTerminal = modelData.name
                                    }
                                    background: Rectangle {
                                        radius: 8
                                        color: tabButton.selected ? "#17313A" : "#121C26"
                                        border.color: tabButton.selected ? "#28545C" : "#1E2C39"
                                    }
                                    contentItem: RowLayout {
                                        spacing: 6
                                        StatusDot {
                                            visible: tabButton.modelData.name !== ""
                                            healthy: tabButton.modelData.state === "active"
                                        }
                                        Label {
                                            text: tabButton.text
                                            color: tabButton.selected ? "#67DFE7" : "#93A7B8"
                                            leftPadding: tabButton.modelData.name === "" ? 6 : 0; rightPadding: 6
                                        }
                                    }
                                }
                            }
                            Item { Layout.fillWidth: true }
                            CheckBox { id: followLog; text: "Follow"; checked: true }
                            CalmButton {
                                visible: root.selectedTerminal !== ""
                                text: "Open log file"
                                onClicked: root.controller.openProcessLog(root.selectedTerminal)
                            }
                            CalmButton {
                                text: "Clear view"
                                onClicked: {
                                    root.controller.uiEvent("clear_log_view", root.selectedTerminal || "Events")
                                    if (root.selectedTerminal === "") logModel.clear()
                                    else terminalView.model.clear()
                                }
                            }
                        }
                        Panel {
                            Layout.fillWidth: true; Layout.fillHeight: true
                            visible: root.selectedTerminal === ""
                            ListView {
                                id: eventsView
                                anchors.fill: parent; anchors.margins: 8; clip: true; model: logModel; spacing: 2
                                onCountChanged: if (followLog.checked) positionViewAtEnd()
                                ScrollBar.vertical: ScrollBar {}
                                delegate: Rectangle {
                                    required property int index
                                    required property string timestamp
                                    required property string subsystem
                                    required property string severity
                                    required property string message
                                    width: ListView.view.width; height: Math.max(36, logText.implicitHeight + 14); color: index % 2 ? "#111A24" : "#131E29"
                                    RowLayout {
                                        anchors.fill: parent; anchors.margins: 7
                                        Label { text: timestamp.slice(11); color: "#62798B"; font.family: "monospace"; Layout.preferredWidth: 72 }
                                        Label { text: subsystem; color: "#57CAD3"; font.bold: true; Layout.preferredWidth: 90 }
                                        Label { id: logText; text: message; color: severity === "error" ? "#FF8992" : (severity === "warning" ? "#E8C468" : "#C8D7E0"); wrapMode: Text.Wrap; Layout.fillWidth: true }
                                    }
                                }
                            }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.fillHeight: true
                            visible: root.selectedTerminal !== ""
                            color: "#070C11"; radius: 10; border.color: "#1E2C39"
                            ListView {
                                id: terminalView
                                anchors.fill: parent; anchors.margins: 8; clip: true
                                model: root.selectedTerminal !== "" ? root.controller.terminalModel(root.selectedTerminal) : null
                                reuseItems: true
                                onCountChanged: if (followLog.checked) positionViewAtEnd()
                                ScrollBar.vertical: ScrollBar {}
                                ScrollBar.horizontal: ScrollBar {}
                                contentWidth: width * 2
                                flickableDirection: Flickable.AutoFlickDirection
                                delegate: Row {
                                    required property string time
                                    required property string html
                                    required property string severity
                                    spacing: 10
                                    Text { text: time; color: "#4F6576"; font.family: "monospace"; font.pixelSize: 12 }
                                    Text {
                                        text: html; textFormat: Text.StyledText
                                        color: severity === "error" ? "#FF8992" : (severity === "warning" ? "#E8C468" : "#C8D7E0")
                                        font.family: "monospace"; font.pixelSize: 12
                                    }
                                }
                            }
                        }
                    }
                }

                // Processes
                Item {
                    Timer {
                        interval: 3000; repeat: true; triggeredOnStart: true
                        running: root.pageNames[root.currentPage] === "Processes"
                        onTriggered: root.controller.refreshProcesses()
                    }
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 22; spacing: 12
                        RowLayout {
                            Layout.fillWidth: true; spacing: 10
                            ColumnLayout {
                                Layout.fillWidth: true; spacing: 2
                                SectionTitle { text: "Everything DracoViLoc is running on this machine" }
                                MutedText { Layout.fillWidth: true; wrapMode: Text.Wrap; text: "Includes processes started from a terminal. Stop = Ctrl+C, then SIGTERM, then SIGKILL, with children; Kill = SIGKILL now. Anything that survives is listed below with the reason." }
                            }
                            CalmButton { text: "Open RViz"; onClicked: root.controller.openRviz() }
                            SafetyButton {
                                text: "Clean up everything"
                                onClicked: root.controller.cleanUpEverything()
                            }
                        }
                        Rectangle {
                            Layout.fillWidth: true
                            visible: (root.processList.report || []).length > 0
                            implicitHeight: reportColumn.implicitHeight + 16
                            radius: 9; color: "#121C26"; border.color: "#1E2C39"
                            ColumnLayout {
                                id: reportColumn
                                anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 8
                                Repeater {
                                    model: root.processList.report || []
                                    delegate: Label {
                                        required property string modelData
                                        Layout.fillWidth: true; wrapMode: Text.Wrap; font.pixelSize: 12; font.family: "monospace"
                                        text: modelData
                                        color: modelData.indexOf("STILL RUNNING") >= 0 || modelData.indexOf("failed") >= 0 ? "#FF8992" : "#5BD08A"
                                    }
                                }
                            }
                        }
                        MutedText {
                            visible: (root.processList.items || []).length === 0
                            text: root.processList.scanning ? "Scanning…" : "Nothing DracoViLoc-related is running."
                        }
                        ScrollView {
                            Layout.fillWidth: true; Layout.fillHeight: true
                            contentWidth: availableWidth
                            ColumnLayout {
                                width: parent.width; spacing: 8
                                Repeater {
                                    model: root.processList.items || []
                                    delegate: Rectangle {
                                        id: processCard
                                        required property var modelData
                                        property bool running: modelData.state !== "stopped"
                                        property bool expanded: false
                                        Layout.fillWidth: true
                                        implicitHeight: processColumn.implicitHeight + 20
                                        radius: 10; color: "#131E29"
                                        border.color: modelData.managed ? "#1E2C39" : "#5A4A2A"
                                        ColumnLayout {
                                            id: processColumn
                                            anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 10
                                            spacing: 4
                                            RowLayout {
                                                Layout.fillWidth: true; spacing: 10
                                                StatusDot { healthy: processCard.running }
                                                ColumnLayout {
                                                    Layout.fillWidth: true; spacing: 1
                                                    Label {
                                                        text: processCard.modelData.name + (processCard.modelData.pid ? "  · pid " + processCard.modelData.pid : "")
                                                        color: "#EEF5F8"; font.bold: true
                                                    }
                                                    MutedText {
                                                        Layout.fillWidth: true; elide: Text.ElideRight
                                                        text: (processCard.modelData.kind === "container" ? "Docker container" : (processCard.modelData.kind === "unit" ? "Started by this GUI" : (processCard.modelData.managed ? "Part of a GUI unit" : "Started outside the GUI")))
                                                              + " · " + processCard.modelData.state
                                                              + (processCard.modelData.children && processCard.modelData.children.length ? " · " + processCard.modelData.children.length + " processes" : "")
                                                              + (processCard.modelData.memory_mb ? " · " + root.fmt(processCard.modelData.memory_mb, 0) + " MB" : "")
                                                        color: processCard.modelData.managed ? "#8FA5B8" : "#E8C468"
                                                    }
                                                }
                                                CalmButton {
                                                    visible: (processCard.modelData.children || []).length > 0
                                                    text: processCard.expanded ? "Hide" : "Show processes"
                                                    onClicked: processCard.expanded = !processCard.expanded
                                                }
                                                CalmButton {
                                                    visible: processCard.modelData.restartable === true
                                                    text: processCard.running ? "Restart" : "Start again"
                                                    onClicked: root.controller.restartUnit(processCard.modelData.name)
                                                }
                                                CalmButton {
                                                    visible: processCard.running
                                                    text: "Stop"
                                                    onClicked: root.stopItem(processCard.modelData.id, processCard.modelData.name, false)
                                                }
                                                SafetyButton {
                                                    visible: processCard.running
                                                    text: "Kill"
                                                    onClicked: root.stopItem(processCard.modelData.id, processCard.modelData.name, true)
                                                }
                                            }
                                            Repeater {
                                                model: processCard.expanded ? processCard.modelData.children : []
                                                delegate: RowLayout {
                                                    id: childRow
                                                    required property var modelData
                                                    Layout.fillWidth: true; spacing: 8
                                                    Label { text: childRow.modelData.name; color: "#C8D7E0"; font.family: "monospace"; Layout.preferredWidth: 260; elide: Text.ElideRight }
                                                    MutedText { text: "pid " + childRow.modelData.pid + " · " + childRow.modelData.state + " · " + root.fmt(childRow.modelData.memory_mb, 0) + " MB"; Layout.fillWidth: true }
                                                    CalmButton { text: "Stop"; implicitHeight: 32; onClicked: root.stopItem("pid:" + childRow.modelData.pid, childRow.modelData.name, false) }
                                                    SafetyButton { text: "Kill"; implicitHeight: 32; onClicked: root.stopItem("pid:" + childRow.modelData.pid, childRow.modelData.name, true) }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // Setup & builds
                Item {
                    ColumnLayout {
                        anchors.fill: parent; spacing: 12; anchors.margins: 22
                        RowLayout {
                            Layout.fillWidth: true; spacing: 10
                            ColumnLayout {
                                Layout.fillWidth: true; spacing: 2
                                SectionTitle { text: "Setup & builds for this machine" }
                                MutedText { text: root.setup.checking ? "Checking…" : "Each card checks one component; its button runs the same script you would run in a terminal." }
                            }
                            CalmButton { text: "Check again"; enabled: !root.setup.checking; onClicked: root.controller.refreshSetup() }
                            CalmButton { text: "Cancel job"; visible: root.controller.jobActive; onClicked: root.controller.cancelJob() }
                            PrimaryButton {
                                text: "Bring this machine up to date"
                                enabled: !root.controller.jobActive && !root.controller.runActive && !root.setup.checking
                                highlighted: true
                                onClicked: root.controller.updateAll()
                            }
                        }
                        ScrollView {
                            Layout.fillWidth: true; Layout.fillHeight: true; Layout.verticalStretchFactor: 3
                            contentWidth: availableWidth
                            ColumnLayout {
                                width: parent.width; spacing: 8
                                Repeater {
                                    model: root.setup.jobs
                                    delegate: Rectangle {
                                        id: card
                                        required property var modelData
                                        property bool running: root.setup.current === modelData.id
                                        property bool queued: (root.setup.queue || []).indexOf(modelData.id) >= 0
                                        property bool expanded: false
                                        property color stateColor: modelData.state === "ok" ? "#5BD08A" : (modelData.state === "stale" || modelData.state === "missing" ? "#E8C468" : (modelData.state === "blocked" ? "#FF8992" : "#7E8F9C"))
                                        Layout.fillWidth: true
                                        implicitHeight: cardColumn.implicitHeight + 24
                                        radius: 10; color: "#131E29"; border.color: card.running ? "#35C6D0" : "#1E2C39"
                                        ColumnLayout {
                                            id: cardColumn
                                            anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 12
                                            spacing: 4
                                            RowLayout {
                                                Layout.fillWidth: true; spacing: 10
                                                Rectangle { width: 10; height: 10; radius: 5; color: card.stateColor }
                                                ColumnLayout {
                                                    Layout.fillWidth: true; spacing: 1
                                                    Label { text: card.modelData.title; color: "#EEF5F8"; font.bold: true }
                                                    MutedText {
                                                        Layout.fillWidth: true; elide: Text.ElideRight
                                                        text: card.running ? "Running…" : (card.queued ? "Queued" : card.modelData.summary)
                                                        color: card.running ? "#67DFE7" : card.stateColor
                                                    }
                                                }
                                                CalmButton {
                                                    text: card.expanded ? "Hide details" : "Details"
                                                    visible: (card.modelData.details || []).length > 0 || card.modelData.description
                                                    onClicked: { card.expanded = !card.expanded; root.controller.uiEvent("setup_details", card.modelData.id) }
                                                }
                                                CalmButton {
                                                    text: "Output"
                                                    onClicked: { root.setupTerminal = "job-" + card.modelData.id; root.controller.uiEvent("setup_output", card.modelData.id) }
                                                }
                                                PrimaryButton {
                                                    text: card.modelData.action
                                                    enabled: !root.controller.jobActive && !root.controller.runActive && card.modelData.state !== "blocked"
                                                    onClicked: root.controller.runJob(card.modelData.id)
                                                }
                                            }
                                            Repeater {
                                                model: card.modelData.options || []
                                                delegate: CheckBox {
                                                    required property var modelData
                                                    text: modelData.label
                                                    checked: !!(card.modelData.option_values || {})[modelData.name]
                                                    enabled: !card.running
                                                    onToggled: root.controller.setJobOption(card.modelData.id, modelData.name, checked)
                                                }
                                            }
                                            ColumnLayout {
                                                id: fetchBox
                                                property var fetch: root.setup.fetch || ({})
                                                property bool running: card.running
                                                visible: card.modelData.id === "yolo_fetch" && !!fetchBox.fetch.phase
                                                Layout.fillWidth: true; spacing: 4
                                                MutedText {
                                                    Layout.fillWidth: true; elide: Text.ElideRight
                                                    color: fetchBox.running ? "#67DFE7" : (fetchBox.fetch.failed ? "#FF8992" : "#5BD08A")
                                                    text: {
                                                        const f = fetchBox.fetch
                                                        if (f.finished)
                                                            return "Finished: " + f.added + " model(s) added, " + f.exported + " exported" + (f.failed ? ", " + f.failed + " failed (see Output)" : "")
                                                        if (!fetchBox.running)
                                                            return "Fetch stopped before finishing (see Output)"
                                                        if (f.phase === "starting" || f.phase === "listing")
                                                            return "Listing the Google Drive folder…"
                                                        return (f.phase === "download" ? "Downloading " : "Exporting ") + (f.current || "")
                                                    }
                                                }
                                                Repeater {
                                                    model: [{ key: "download", label: "Downloaded" }, { key: "export", label: "Exported" }]
                                                    delegate: RowLayout {
                                                        id: barRow
                                                        required property var modelData
                                                        property var counts: fetchBox.fetch[modelData.key] || [0, 0]
                                                        Layout.fillWidth: true; spacing: 10
                                                        MutedText { text: barRow.modelData.label; Layout.preferredWidth: 90 }
                                                        Rectangle {
                                                            Layout.fillWidth: true; height: 8; radius: 4; color: "#0D1620"; border.color: "#1E2C39"
                                                            Rectangle {
                                                                height: parent.height; radius: 4; color: "#35C6D0"
                                                                width: barRow.counts[1] > 0 ? parent.width * barRow.counts[0] / barRow.counts[1] : (fetchBox.fetch.finished ? parent.width : 0)
                                                                Behavior on width { NumberAnimation { duration: 200 } }
                                                            }
                                                        }
                                                        MutedText {
                                                            Layout.preferredWidth: 70; horizontalAlignment: Text.AlignRight
                                                            property var order: ["starting", "listing", "download", "export", "done"]
                                                            // a phase with nothing to do shows "none new" once it has been reached
                                                            text: barRow.counts[1] > 0 ? barRow.counts[0] + " / " + barRow.counts[1] : (order.indexOf(fetchBox.fetch.phase) >= order.indexOf(barRow.modelData.key) ? "none new" : "–")
                                                        }
                                                    }
                                                }
                                            }
                                            MutedText {
                                                visible: card.expanded; Layout.fillWidth: true; wrapMode: Text.Wrap
                                                text: card.modelData.description
                                            }
                                            Repeater {
                                                model: card.expanded ? (card.modelData.details || []) : []
                                                delegate: Label {
                                                    required property string modelData
                                                    Layout.fillWidth: true; elide: Text.ElideMiddle
                                                    text: modelData; color: "#93A7B8"; font.family: "monospace"; font.pixelSize: 12
                                                }
                                            }
                                        }
                                    }
                                }
                                Rectangle {
                                    Layout.fillWidth: true; implicitHeight: modelsColumn.implicitHeight + 24
                                    radius: 10; color: "#131E29"; border.color: "#1E2C39"
                                    ColumnLayout {
                                        id: modelsColumn
                                        anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 12
                                        spacing: 6
                                        RowLayout {
                                            Layout.fillWidth: true
                                            Label { text: "Visual models (YOLO, D-FINE)"; color: "#EEF5F8"; font.bold: true }
                                            MutedText { text: root.setup.gpu ? "plans for " + root.setup.gpu : "no NVIDIA GPU: plans cannot be built here" }
                                            Item { Layout.fillWidth: true }
                                            MutedText {
                                                property var autos: root.setup.autos || ({})
                                                text: "Auto: " + [["yolo", "YOLO"], ["dfine", "D-FINE"], ["dfine_n", "D-FINE-N"]].map(d => d[1] + " " + (autos[d[0]] || "none ready")).join(" · ")
                                                color: autos.yolo ? "#5BD08A" : "#E8C468"
                                            }
                                        }
                                        MutedText { Layout.fillWidth: true; wrapMode: Text.Wrap; text: "Checkpoint (.pt) → ONNX export (portable, models/yolo) → TensorRT plan built on this GPU (Isaac workspace). Plan builds take about 10 minutes on a Jetson Orin." }
                                        Repeater {
                                            model: root.setup.models || []
                                            delegate: RowLayout {
                                                id: modelRow
                                                required property var modelData
                                                Layout.fillWidth: true; spacing: 10
                                                ColumnLayout {
                                                    Layout.fillWidth: true; spacing: 0
                                                    Label {
                                                        text: modelRow.modelData.id + (modelRow.modelData.legacy ? "  (legacy)" : "") + (root.profile.yolo_model === modelRow.modelData.id ? "  · selected" : (root.setup.auto === modelRow.modelData.id ? "  · auto" : ""))
                                                        color: "#D9E6EE"; font.family: "monospace"; font.pixelSize: 13
                                                    }
                                                    MutedText {
                                                        Layout.fillWidth: true; elide: Text.ElideRight
                                                        text: modelRow.modelData.date + " · ONNX: " + modelRow.modelData.onnx_detail + " · Plan: " + modelRow.modelData.plan_detail
                                                        color: modelRow.modelData.ready ? "#5BD08A" : (modelRow.modelData.onnx_state === "stale" || modelRow.modelData.plan_state === "stale" ? "#E8C468" : "#8FA5B8")
                                                    }
                                                }
                                                CalmButton {
                                                    text: modelRow.modelData.onnx_state === "ok" ? "Re-export" : "Export ONNX"
                                                    visible: modelRow.modelData.has_checkpoint
                                                    enabled: !root.controller.jobActive && !root.controller.runActive
                                                    onClicked: root.controller.exportModel(modelRow.modelData.id)
                                                }
                                                CalmButton {
                                                    text: modelRow.modelData.plan_state === "ok" ? "Rebuild plan" : "Build plan"
                                                    visible: !modelRow.modelData.legacy
                                                    enabled: !root.controller.jobActive && !root.controller.runActive && modelRow.modelData.onnx_state === "ok" && modelRow.modelData.plan_state !== "unavailable"
                                                    onClicked: root.controller.buildModelPlan(modelRow.modelData.id)
                                                }
                                                CalmButton {
                                                    text: "Use for runs"
                                                    enabled: modelRow.modelData.ready && !root.controller.runActive && root.profile.yolo_model !== modelRow.modelData.id
                                                    onClicked: root.controller.setOption("yolo_model", modelRow.modelData.id)
                                                }
                                            }
                                        }
                                        CalmButton {
                                            text: "Use Auto (newest ready)"
                                            visible: root.profile.yolo_model !== "auto"
                                            enabled: !root.controller.runActive
                                            onClicked: root.controller.setOption("yolo_model", "auto")
                                        }
                                    }
                                }
                                Rectangle {
                                    Layout.fillWidth: true; implicitHeight: imageColumn.implicitHeight + 24
                                    radius: 10; color: "#131E29"; border.color: "#1E2C39"
                                    ColumnLayout {
                                        id: imageColumn
                                        anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 12
                                        spacing: 6
                                        Label { text: "Move the project image between machines of the same CPU type"; color: "#EEF5F8"; font.bold: true }
                                        MutedText { Layout.fillWidth: true; wrapMode: Text.Wrap; text: "Save writes a compressed copy (many GB) to " + (root.setup.images_directory || "images/") + ". Copy that file into the same folder on the other machine, then load it there." }
                                        RowLayout {
                                            spacing: 8
                                            CalmButton { text: "Save image"; enabled: !root.controller.jobActive && !root.controller.runActive; onClicked: root.controller.saveProjectImage() }
                                            ComboBox { id: savedImages; Layout.preferredWidth: 360; model: root.setup.saved_images || [] }
                                            CalmButton { text: "Load selected"; enabled: savedImages.count > 0 && !root.controller.jobActive && !root.controller.runActive; onClicked: root.controller.loadProjectImage(savedImages.currentText) }
                                        }
                                    }
                                }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            SectionTitle { text: root.setupTerminal ? "Output: " + root.setupTerminal.replace("job-", "") : "Output" }
                            Item { Layout.fillWidth: true }
                            CheckBox { id: followSetup; text: "Follow"; checked: true }
                            CalmButton { text: "Open log file"; visible: root.setupTerminal !== ""; onClicked: root.controller.openProcessLog(root.setupTerminal) }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.fillHeight: true; Layout.verticalStretchFactor: 2; Layout.minimumHeight: 160
                            color: "#070C11"; radius: 10; border.color: "#1E2C39"
                            MutedText { anchors.centerIn: parent; visible: root.setupTerminal === ""; text: "Run a job or press Output on a card to see its terminal." }
                            ListView {
                                anchors.fill: parent; anchors.margins: 8; clip: true
                                model: root.setupTerminal !== "" ? root.controller.terminalModel(root.setupTerminal) : null
                                reuseItems: true
                                onCountChanged: if (followSetup.checked) positionViewAtEnd()
                                ScrollBar.vertical: ScrollBar {}
                                delegate: Row {
                                    required property string time
                                    required property string html
                                    required property string severity
                                    spacing: 10
                                    Text { text: time; color: "#4F6576"; font.family: "monospace"; font.pixelSize: 12 }
                                    Text {
                                        text: html; textFormat: Text.StyledText
                                        color: severity === "error" ? "#FF8992" : (severity === "warning" ? "#E8C468" : "#C8D7E0")
                                        font.family: "monospace"; font.pixelSize: 12
                                    }
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
                                    MutedText { text: "ROS Humble · domain 42 · engines and images chosen for this machine" }
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
                                MutedText { text: root.profile.dual_arm ? "This application can request controlled ROS motion stops for both cobots. It is not safety-rated and cannot replace either FAIRINO emergency stop, controller safeguards, or clear workspaces." : "This application can request controlled ROS motion stops. It is not safety-rated and cannot replace the FAIRINO emergency stop, controller safeguards, or a clear workspace." }
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
        id: closeDialog; modal: true; anchors.centerIn: parent; title: "Close control center?"
        onOpened: root.controller.uiEvent("dialog_opened", title)
        onRejected: root.controller.uiEvent("dialog_cancelled", title)
        width: 500
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: root.controller.requestQuit()
        contentItem: MutedText { text: root.profile.execution_target === "remote" ? "The Jetson run—including tracking—will continue. This window will release its controller lease and close." : "An active run is managed by this window. Confirm to stop motion, finalize recordings, shut down managed processes, and exit."; width: 430; color: root.profile.execution_target === "remote" ? "#D8A65A" : "#8FA5B8" }
    }
    Dialog {
        id: forceStopDialog; modal: true; anchors.centerIn: parent; title: "Confirm physical emergency stop"
        onOpened: root.controller.uiEvent("dialog_opened", title)
        onRejected: root.controller.uiEvent("dialog_cancelled", title)
        width: 520
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: root.controller.forceStopAfterEstop()
        contentItem: MutedText { text: root.profile.dual_arm ? "Only continue if both physical FAIRINO emergency stops are engaged and both cobots are mechanically safe. This will terminate the control processes without a confirmed ROS hold." : "Only continue if the physical FAIRINO emergency stop is engaged and motion is mechanically safe. This will terminate the control processes without a confirmed ROS hold."; width: 450; color: "#FF9AA2" }
    }
}
