package com.example.myapplication

import android.Manifest
import android.app.AlertDialog
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Bundle
import android.provider.Settings
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.selection.SelectionContainer
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.lifecycle.ViewModelProvider
import com.example.myapplication.probe.*
import com.example.myapplication.remote.RecordingService
import com.example.myapplication.remote.RemoteSnapshot
import com.example.myapplication.remote.RemoteState
import com.example.myapplication.ui.theme.MyApplicationTheme

class MainActivity : ComponentActivity() {
    private lateinit var model: ProbeViewModel
    private val permission = registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
        if (granted) model.inspect()
        else if (!shouldShowRequestPermissionRationale(Manifest.permission.CAMERA)) {
            AlertDialog.Builder(this).setTitle("カメラ権限が必要です")
                .setMessage("端末設定でこのアプリのカメラ権限を許可すると、機能調査と試験録画を利用できます。")
                .setPositiveButton("設定を開く") { _, _ ->
                    startActivity(Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS, Uri.parse("package:$packageName")))
                }.setNegativeButton("閉じる", null).show()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        model = ViewModelProvider(this)[ProbeViewModel::class.java]
        enableEdgeToEdge()
        setContent {
            MyApplicationTheme {
                ProbeScreen(model.state, RemoteState.state, ::connectRemote, ::disconnectRemote, ::inspectWithPermission, { camera, encoder, seconds, allow ->
                    if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                        model.start(camera, encoder, seconds, allow)
                    } else inspectWithPermission()
                }, model::stop)
            }
        }
    }

    override fun onStart() {
        super.onStart()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
    }

    override fun onStop() {
        // Phase 1 is a visible-Activity experiment. A later recording service will own background recording.
        if (!isChangingConfigurations) model.stop()
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        super.onStop()
    }

    private fun connectRemote(base: String, token: String) {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            inspectWithPermission()
            return
        }
        try {
            ContextCompat.startForegroundService(this, Intent(this, RecordingService::class.java)
                .putExtra("base_url", base.trim().trimEnd('/')).putExtra("token", token.trim()))
        } catch (error: Exception) {
            AlertDialog.Builder(this).setMessage("PC接続を開始できません: $error").setPositiveButton("閉じる", null).show()
        }
    }

    private fun disconnectRemote() {
        startService(Intent(this, RecordingService::class.java).setAction(RecordingService.ACTION_STOP))
    }

    private fun inspectWithPermission() {
        when {
            ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED -> model.inspect()
            shouldShowRequestPermissionRationale(Manifest.permission.CAMERA) ->
                AlertDialog.Builder(this).setTitle("カメラの使用")
                    .setMessage("背面カメラの対応機能を調べ、最大30秒の試験動画をアプリ専用領域に保存します。")
                    .setPositiveButton("続ける") { _, _ -> permission.launch(Manifest.permission.CAMERA) }
                    .setNegativeButton("キャンセル", null).show()
            else -> permission.launch(Manifest.permission.CAMERA)
        }
    }
}

@Composable
private fun ProbeScreen(
    state: ProbeUiState, remote: RemoteSnapshot, onConnect: (String, String) -> Unit, onDisconnect: () -> Unit,
    onInspect: () -> Unit,
    onStart: (String?, String?, Int, Boolean) -> Unit, onStop: () -> Unit
) {
    val context = androidx.compose.ui.platform.LocalContext.current
    val preferences = remember { context.getSharedPreferences("remote", android.content.Context.MODE_PRIVATE) }
    var baseUrl by rememberSaveable { mutableStateOf(preferences.getString("base_url", "http://127.0.0.1:8765") ?: "") }
    var token by rememberSaveable { mutableStateOf(preferences.getString("token", "") ?: "") }
    var cameraId by rememberSaveable { mutableStateOf<String?>(null) }
    var encoderName by rememberSaveable { mutableStateOf<String?>(null) }
    var durationText by rememberSaveable { mutableStateOf("10") }
    var allowUndeclared by rememberSaveable { mutableStateOf(false) }
    var showJson by rememberSaveable { mutableStateOf(false) }
    val report = state.report
    LaunchedEffect(report) {
        if (report != null) {
            cameraId = report.cameras.firstOrNull { it.id == cameraId }?.id
                ?: report.cameras.firstOrNull { it.targetSupported }?.id ?: report.cameras.firstOrNull()?.id
            encoderName = report.encoders.firstOrNull { it.name == encoderName }?.name
                ?: report.encoders.firstOrNull { it.surfaceInput && it.sizeAndRateSupported && it.formatSupported }?.name
                ?: report.encoders.firstOrNull { it.surfaceInput }?.name
        }
    }
    val camera = report?.cameras?.firstOrNull { it.id == cameraId }
    val encoder = report?.encoders?.firstOrNull { it.name == encoderName }
    val duration = durationText.toIntOrNull()?.takeIf { it in 1..30 }
    val canStart = camera?.targetSupported == true && encoder?.surfaceInput == true && duration != null &&
        (allowUndeclared || (encoder.sizeAndRateSupported && encoder.formatSupported))
    Scaffold(modifier = Modifier.fillMaxSize()) { padding ->
        Column(Modifier.padding(padding).fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp)) {
            Text("Pixel Camera Probe", style = MaterialTheme.typography.headlineMedium)
            Text("USB遠隔録画 · 1280 × 720 / 固定240 fps / H.264", style = MaterialTheme.typography.titleMedium)
            Text("APIの対応申告と録画実測を別々に検査します。実際の240 fps達成は、動く時刻基準も撮影して確認してください。")
            Card(Modifier.fillMaxWidth()) {
                Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                    Text("PCから録画する", style = MaterialTheme.typography.titleLarge)
                    Text(if (remote.connected) "PC接続済み" else if (remote.enabled) "PCへ接続中" else "PC接続は停止中")
                    Text(remote.message)
                    Text("状態: ${remote.phase} / 撮影: %.2f 秒 / 未転送: ${remote.pending}件".format(remote.elapsed))
                    if (!remote.enabled) {
                        OutlinedTextField(value = token, onValueChange = { token = java.text.Normalizer.normalize(it, java.text.Normalizer.Form.NFKC).trim().lowercase() }, label = { Text("PCに表示された接続コード") },
                            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Ascii, autoCorrectEnabled = false),
                            singleLine = true, modifier = Modifier.fillMaxWidth())
                        OutlinedTextField(value = baseUrl, onValueChange = { baseUrl = it }, label = { Text("PC接続先（USBは変更不要）") },
                            singleLine = true, modifier = Modifier.fillMaxWidth())
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Button(onClick = { onConnect(baseUrl, token) }, enabled = !remote.enabled && !state.busy && token.matches(Regex("[0-9a-f]{32}"))) {
                            Text("PC接続を開始")
                        }
                        OutlinedButton(onClick = onDisconnect, enabled = remote.enabled) { Text("PC接続を停止") }
                    }
                    if (remote.phase == "TRANSFERRING") {
                        Text("転送: ${remote.transferPercent}%")
                    }
                    if (remote.lastError.isNotEmpty()) Text(remote.lastError, color = MaterialTheme.colorScheme.error)
                    Text("PC保存とSHA-256照合が成功した遠隔録画だけ、端末の動画を削除します。以前の試験動画は保持します。")
                }
            }
            if (!remote.enabled) {
                Card(Modifier.fillMaxWidth()) {
                    Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                        Text(state.status, style = MaterialTheme.typography.titleMedium)
                        Text("撮影経過: %.2f 秒".format(state.elapsedSeconds))
                        Text("空き容量: %.2f GiB".format(state.freeBytes / 1073741824.0))
                        if (state.busy) LinearProgressIndicator(Modifier.fillMaxWidth())
                    }
                }
                Button(onClick = onInspect, enabled = !state.busy && !remote.enabled) { Text("権限を許可・機能を調査") }
                if (report != null) {
                    val device = report.json.getJSONObject("device")
                    Text("${device.optString("manufacturer")} ${device.optString("model")} / Android ${device.optString("android_release")} (API ${device.optInt("sdk")})")
                    Text("背面カメラ", style = MaterialTheme.typography.titleMedium)
                    if (report.cameras.isEmpty()) Text("利用可能な背面カメラがありません")
                    report.cameras.forEach { option ->
                        Row(Modifier.fillMaxWidth()) {
                            RadioButton(selected = cameraId == option.id, enabled = !state.busy && !remote.enabled, onClick = { cameraId = option.id })
                            Text(option.label, Modifier.padding(top = 12.dp).weight(1f))
                        }
                    }
                    if (camera != null && !camera.targetSupported) {
                        Text("このカメラには720p [240,240]の申告がありません。詳細JSONで120 fpsなどの対応モードを確認してください。",
                            color = MaterialTheme.colorScheme.error)
                    }
                    Text("H.264エンコーダ", style = MaterialTheme.typography.titleMedium)
                    if (report.encoders.isEmpty()) Text("H.264エンコーダがありません")
                    report.encoders.forEach { option ->
                        Row(Modifier.fillMaxWidth()) {
                            RadioButton(selected = encoderName == option.name, enabled = !state.busy && !remote.enabled && option.surfaceInput,
                                onClick = { encoderName = option.name })
                            Text("${option.name}\n${if (option.hardware) "ハードウェア" else "ソフトウェア"} / Surface: ${option.surfaceInput}\n" +
                                "720p240申告: ${option.sizeAndRateSupported} / format: ${option.formatSupported}\n" +
                                "試験ビットレート: ${option.bitrate / 1_000_000.0} Mbps", Modifier.weight(1f).padding(top = 8.dp))
                        }
                    }
                    Row {
                        Checkbox(checked = allowUndeclared, enabled = !state.busy && !remote.enabled, onCheckedChange = { allowUndeclared = it })
                        Text("エンコーダの対応申告がない場合も試験する\n成功や240 fps達成を保証する設定ではありません",
                            Modifier.weight(1f).padding(top = 8.dp))
                    }
                    Text("AE・AF・AWB: 自動。ISO・露光時間・手動フォーカス・WB・EV変更はPhase 1では無効です。")
                    OutlinedTextField(value = durationText, onValueChange = { durationText = it }, enabled = !state.busy && !remote.enabled,
                        label = { Text("自動停止までの秒数 (1〜30)") }, singleLine = true,
                        isError = duration == null, modifier = Modifier.fillMaxWidth())
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Button(onClick = { onStart(cameraId, encoderName, duration ?: 10, allowUndeclared) }, enabled = !state.busy && !remote.enabled && canStart) {
                            Text("720p240 試験録画")
                        }
                        OutlinedButton(onClick = onStop, enabled = state.recordingTask) { Text("停止") }
                    }
                    Text("この下の試験録画は画面を開いたまま使います。PC接続中の録画はサービスが管理します。")
                    Text("プレビューと音声は使用しません。下の手動試験録画は端末に保持します。")
                }
                if (state.result.isNotEmpty()) {
                    Text("試験結果", style = MaterialTheme.typography.titleMedium)
                    SelectionContainer { Text(state.result) }
                }
                if (state.capabilityPath.isNotEmpty()) {
                    SelectionContainer { Text("機能レポート: ${state.capabilityPath}\n録画レポート: ${state.metadataPath}") }
                    TextButton(onClick = { showJson = !showJson }) { Text(if (showJson) "詳細JSONを閉じる" else "対応機能の詳細JSONを表示") }
                    if (showJson) SelectionContainer { Text(report?.json?.toString(2).orEmpty(), style = MaterialTheme.typography.bodySmall) }
                }
            } else {
                Text("PC画面またはPythonから録画できます。端末空き容量: %.2f GiB".format(remote.freeBytes / 1073741824.0))
            }
            Spacer(Modifier.height(16.dp))
        }
    }
}
