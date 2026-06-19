
package com.esp32surv

import android.Manifest
import android.annotation.SuppressLint
import android.content.ContentValues
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Build
import android.os.Bundle
import android.os.Environment
import android.provider.MediaStore
import android.util.Base64
import android.util.Log
import android.view.View
import android.view.WindowManager
import android.webkit.*
import android.widget.Toast
import androidx.activity.OnBackPressedCallback
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import kotlinx.coroutines.*
import org.json.JSONArray
import java.io.*
import java.text.SimpleDateFormat
import java.util.*

class MainActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "ESP32Surv"
        private const val SAVE_DIR = "ESP32Surveillance"
    }

    private lateinit var webView: WebView
    private val mainScope = CoroutineScope(Dispatchers.Main + SupervisorJob())

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val allGranted = permissions.values.all { it }
        if (!allGranted) {
            Toast.makeText(this, "Storage permission needed", Toast.LENGTH_LONG).show()
        }
    }

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // ✅ Enable debugging
        WebView.setWebContentsDebuggingEnabled(true)

        webView = WebView(this).apply {
            setLayerType(View.LAYER_TYPE_HARDWARE, null)
            setBackgroundColor(0x00000000)
        }
        setContentView(webView)
        hideSystemUI()

        webView.settings.apply {
            javaScriptEnabled = true
            domStorageEnabled = true
            allowFileAccess = true
            allowContentAccess = true
            mediaPlaybackRequiresUserGesture = false

            // ✅ Performance
            cacheMode = WebSettings.LOAD_NO_CACHE
            databaseEnabled = true
            setRenderPriority(WebSettings.RenderPriority.HIGH)

            // ✅ FIX: Allow ESP32 HTTP + WS
            mixedContentMode = WebSettings.MIXED_CONTENT_ALWAYS_ALLOW
        }

        webView.webViewClient = WebViewClient()
        webView.webChromeClient = WebChromeClient()

        webView.addJavascriptInterface(AndroidBridge(), "AndroidBridge")

        webView.loadUrl("file:///android_asset/esp32_surveillance_dashboard.html")

        onBackPressedDispatcher.addCallback(this, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() {
                if (webView.canGoBack()) webView.goBack() else finish()
            }
        })

        requestStoragePermissions()
    }

    private fun hideSystemUI() {
        WindowCompat.setDecorFitsSystemWindows(window, false)
        webView.post {
            WindowCompat.getInsetsController(window, webView).apply {
                hide(WindowInsetsCompat.Type.systemBars())
                systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
            }
        }
    }

    private fun requestStoragePermissions() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) {
            val perms = arrayOf(
                Manifest.permission.WRITE_EXTERNAL_STORAGE,
                Manifest.permission.READ_EXTERNAL_STORAGE
            )
            permissionLauncher.launch(perms)
        }
    }

    inner class AndroidBridge {

        @JavascriptInterface
        fun saveImage(base64Data: String, filename: String) {
            mainScope.launch(Dispatchers.IO) {
                try {
                    // ✅ FIX: remove data:image/jpeg;base64,
                    val cleanBase64 = base64Data.substringAfter(",")

                    val bytes = Base64.decode(cleanBase64, Base64.DEFAULT)
                    val bitmap = BitmapFactory.decodeByteArray(bytes, 0, bytes.size)

                    if (bitmap != null) {
                        saveBitmapToGallery(bitmap, filename)
                        showToast("Image saved")
                    }
                } catch (e: Exception) {
                    showToast("Save failed")
                }
            }
        }

        @JavascriptInterface
        fun saveVideoFrames(jsonArray: String, filename: String) {
            mainScope.launch(Dispatchers.IO) {
                try {
                    val arr = JSONArray(jsonArray)
                    val ts = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())

                    for (i in 0 until arr.length()) {
                        val base64 = arr.getString(i)

                        // ✅ FIX HERE ALSO
                        val cleanBase64 = base64.substringAfter(",")

                        val bytes = Base64.decode(cleanBase64, Base64.DEFAULT)
                        val bmp = BitmapFactory.decodeByteArray(bytes, 0, bytes.size)

                        if (bmp != null) {
                            val name = "REC_${ts}_$i.jpg"
                            saveBitmapToGallery(bmp, name, "ESP32Recordings/$ts")
                        }
                    }

                    showToast("Recording saved")

                } catch (e: Exception) {
                    showToast("Video save error")
                }
            }
        }

        @JavascriptInterface
        fun showNativeToast(msg: String) {
            showToast(msg)
        }

        @JavascriptInterface
        fun isAndroid(): Boolean = true
    }

    private fun saveBitmapToGallery(bitmap: Bitmap, filename: String, subDir: String = SAVE_DIR): Boolean {
        return try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                val values = ContentValues().apply {
                    put(MediaStore.Images.Media.DISPLAY_NAME, filename)
                    put(MediaStore.Images.Media.MIME_TYPE, "image/jpeg")
                    put(MediaStore.Images.Media.RELATIVE_PATH, "${Environment.DIRECTORY_PICTURES}/$subDir")
                }

                val uri = contentResolver.insert(MediaStore.Images.Media.EXTERNAL_CONTENT_URI, values)
                uri?.let {
                    contentResolver.openOutputStream(it)?.use { stream ->
                        bitmap.compress(Bitmap.CompressFormat.JPEG, 90, stream)
                    }
                }
                true
            } else {
                val dir = File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES), subDir)
                if (!dir.exists()) dir.mkdirs()

                val file = File(dir, filename)
                FileOutputStream(file).use {
                    bitmap.compress(Bitmap.CompressFormat.JPEG, 90, it)
                }
                true
            }
        } catch (e: Exception) {
            false
        }
    }

    private fun showToast(msg: String) {
        mainScope.launch(Dispatchers.Main) {
            Toast.makeText(this@MainActivity, msg, Toast.LENGTH_SHORT).show()
        }
    }
}

