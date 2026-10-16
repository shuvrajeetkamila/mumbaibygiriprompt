// =============================================================================
//  MUMBAI 1:1 EXACT — android/app/src/main/java/com/mumbai/exact/WorldFetch.java
//
//  THE ONLY JAVA IN THE APP, AND THE REASON FOR IT IS HTTPS.
//
//  The engine has no TLS stack and will not grow one; Android has one, maintained,
//  with a certificate store that is actually updated.  So the city download's
//  transport is a byte-range GET performed here, and the bytes go straight into the
//  C++ state machine (engine/world/world_download.h) one chunk at a time.
//
//  WHAT THIS CLASS DOES NOT DO.  It does not verify anything: the checksum list and
//  the SHA-256 of every file are the engine's business, because a transport that
//  also decides what is authentic is a transport that can be trusted to lie.  It
//  does not retry, resume, or keep state between calls: the downloader above it
//  knows where it stopped and what it still needs, and a second opinion here would
//  be a second source of truth about the world on disk.
//
//  THE RETURN VALUE IS THE WHOLE CONTRACT (see engine/app/android_http.cpp):
//
//    >= 0   the range arrived; the number is the resource's total size in bytes
//    -1     it failed, and lastError() says why
//    -2     the C++ side said "enough for this step"; lastTotal() is the size
//    -3     the server ignored the Range header and sent the whole file from zero
//
//  THE TWO STATICS ARE NOT LAZY THREAD SAFETY.  The app runs one thread: the looper
//  pumps the lifecycle, this class blocks on the network, and the C++ side parses in
//  the same call.  There is no second caller to race with, and inventing a lock here
//  would hide the fact that a download is the frame loop's own work this step.
// =============================================================================
package com.mumbai.exact;

import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;

public final class WorldFetch {

    static {
        System.loadLibrary("mumbai");
    }

    /** One chunk per JNI call.  64 KiB is large enough to keep the bridge cheap and
     *  small enough that the engine can copy it out without a second buffer. */
    private static final int kChunk = 64 * 1024;

    private static final byte[] buffer = new byte[kChunk];

    private static String lastError = "";
    private static long lastTotal = 0;

    private WorldFetch() {}

    /**
     * Called from native code for every chunk of the body.  Returning false means
     * "stop": the C++ side has spent the step's byte budget and will come back for
     * the rest of the range on a later frame.
     */
    private static native boolean onChunk(long handle, byte[] data, int length);

    /** Why the last get() returned -1.  Never null; empty when there is nothing to say. */
    /**
     * WHAT DEVICE THIS IS, IN ONE LINE.
     *
     * It exists because a phone that shows a black screen cannot be debugged from
     * the machine that built the app: the log lives on the device, and the person
     * holding it is not going to run adb.  So the app asks three questions it
     * cannot answer in C++ — the model, the Android version, and whether this
     * device can do Vulkan 1.1 at all — and puts the answer on the screen and in
     * the log file below (D-054).
     *
     * hasSystemFeature(name, version) is the platform's own answer to the Vulkan
     * question, and it is the one the installer uses to decide whether this app may
     * be installed at all.  0x00401000 is Vulkan 1.1.
     *
     * Never throws: a device that cannot answer must not stop the app.
     */
    public static String systemInfo(android.content.Context context) {
        final StringBuilder out = new StringBuilder();
        try {
            out.append(android.os.Build.MANUFACTURER).append(' ').append(android.os.Build.MODEL);
            out.append(" / Android ").append(android.os.Build.VERSION.RELEASE);
            final String[] abis = android.os.Build.SUPPORTED_ABIS;
            out.append(" / ").append(abis.length > 0 ? abis[0] : "unknown abi");
        } catch (Throwable failure) {
            out.append("device info unavailable: ").append(failure);
        }
        try {
            // Which build is this?  The release download link is the same for every
            // build, so this string is the only thing on a phone that can say which
            // one it is running (see build.gradle).
            out.append(" / app ").append(context.getPackageManager()
                    .getPackageInfo(context.getPackageName(), 0).versionName);
        } catch (Throwable failure) {
            out.append(" / app unknown");
        }
        try {
            final android.content.pm.PackageManager manager = context.getPackageManager();
            final boolean vulkan11 = manager.hasSystemFeature(
                    android.content.pm.PackageManager.FEATURE_VULKAN_HARDWARE_VERSION, 0x00401000);
            out.append(" / Vulkan 1.1: ").append(vulkan11 ? "yes" : "NO");
        } catch (Throwable failure) {
            out.append(" / Vulkan 1.1: unknown");
        }
        return out.toString();
    }

    /**
     * THE LOG FILE A PERSON CAN SEND.
     *
     * Written to Downloads, because that is the one place every file manager offers
     * and every share sheet can reach — no cable, no adb, no developer options.  A
     * file manager on a modern Android cannot easily browse the app's own private
     * directories, which is exactly where a log would otherwise be hidden.  If the
     * Downloads collection refuses (an unusual device, a policy), this falls back to
     * the app's own external files directory and returns that path instead.
     *
     * Each write replaces the previous file, so what is on the phone is the newest
     * log and never a stale one wearing the same name.  Returns where it went, or a
     * sentence saying it could not; never throws.
     */
    public static String writeLog(android.content.Context context, String text) {
        final byte[] bytes = text.getBytes(java.nio.charset.StandardCharsets.UTF_8);
        final String name = "mumbai-log.txt";
        try {
            final android.content.ContentResolver resolver = context.getContentResolver();
            // Remove any earlier copy, so the file a person sends is this one.  A
            // refused delete is not a failed write: the new file then lands under a
            // suffixed name, and the newest is still the newest.
            try {
                final android.database.Cursor existing = resolver.query(
                        android.provider.MediaStore.Downloads.EXTERNAL_CONTENT_URI,
                        new String[]{android.provider.MediaStore.MediaColumns._ID},
                        android.provider.MediaStore.MediaColumns.DISPLAY_NAME + "=?",
                        new String[]{name}, null);
                if (existing != null) {
                    while (existing.moveToNext()) {
                        final android.net.Uri doomed = android.content.ContentUris.withAppendedId(
                                android.provider.MediaStore.Downloads.EXTERNAL_CONTENT_URI,
                                existing.getLong(0));
                        resolver.delete(doomed, null, null);
                    }
                    existing.close();
                }
            } catch (Throwable ignored) {
                // ignored on purpose: see above
            }
            final android.content.ContentValues values = new android.content.ContentValues();
            values.put(android.provider.MediaStore.MediaColumns.DISPLAY_NAME, name);
            values.put(android.provider.MediaStore.MediaColumns.MIME_TYPE, "text/plain");
            values.put(android.provider.MediaStore.MediaColumns.RELATIVE_PATH,
                       android.os.Environment.DIRECTORY_DOWNLOADS);
            final android.net.Uri uri = resolver.insert(
                    android.provider.MediaStore.Downloads.EXTERNAL_CONTENT_URI, values);
            if (uri != null) {
                final java.io.OutputStream out = resolver.openOutputStream(uri, "rwt");
                if (out != null) {
                    try {
                        out.write(bytes);
                        out.flush();
                    } finally {
                        out.close();
                    }
                }
                return "Downloads/" + name;
            }
        } catch (Throwable failure) {
            // Falls through to the private directory below.
        }
        try {
            final java.io.File directory = context.getExternalFilesDir(null);
            if (directory != null) {
                final java.io.File file = new java.io.File(directory, name);
                final java.io.FileOutputStream out = new java.io.FileOutputStream(file);
                try {
                    out.write(bytes);
                    out.flush();
                } finally {
                    out.close();
                }
                return file.getAbsolutePath();
            }
        } catch (Throwable failure) {
            return "no place to write a log: " + failure;
        }
        return "no place to write a log";
    }

    public static String lastError() {
        return lastError;
    }

    /** The resource's total size, as far as the last get() could tell. */
    public static long lastTotal() {
        return lastTotal;
    }

    /**
     * Fetches `url` from `offset` to the end, handing every chunk to `onChunk`.
     *
     * @param handle passed straight back to the native callback; the C++ side's own
     *               pointer, which this class must never interpret
     * @return the codes documented above
     */
    public static long get(String url, long offset, long handle) {
        HttpURLConnection connection = null;
        InputStream input = null;
        lastError = "";
        lastTotal = 0;
        try {
            connection = (HttpURLConnection) new URL(url).openConnection();
            connection.setRequestMethod("GET");
            // Release assets live on a different host than the release URL, so the
            // redirect has to be followed, and the Range header has to survive it.
            connection.setInstanceFollowRedirects(true);
            connection.setConnectTimeout(15000);
            connection.setReadTimeout(10000);
            connection.setRequestProperty("Accept-Encoding", "identity");
            if (offset > 0) {
                connection.setRequestProperty("Range", "bytes=" + offset + "-");
            }

            final int status = connection.getResponseCode();
            if (status != 200 && status != 206) {
                lastError = "HTTP " + status;
                return -1;
            }
            if (status == 200 && offset > 0) {
                // The server ignored the range.  The body starts at zero, which is
                // not what the caller asked for; say so instead of handing over a
                // stream nobody can place.
                lastError = "the server ignored the byte range";
                return -3;
            }

            long total = connection.getContentLengthLong();
            final String contentRange = connection.getHeaderField("Content-Range");
            if (contentRange != null) {
                final int slash = contentRange.lastIndexOf('/');
                if (slash > 0) {
                    try {
                        total = Long.parseLong(contentRange.substring(slash + 1).trim());
                    } catch (NumberFormatException ignored) {
                        // "*" or a malformed header: fall through with what we have.
                    }
                }
            } else if (status == 206) {
                // No Content-Range and a partial body: the length is of this range.
                total = offset + connection.getContentLengthLong();
            }
            lastTotal = total;

            input = connection.getInputStream();
            for (;;) {
                final int read = input.read(buffer, 0, kChunk);
                if (read < 0) {
                    return total;   // the whole range arrived
                }
                if (!onChunk(handle, buffer, read)) {
                    return -2;      // budget spent; the caller resumes on a later frame
                }
            }
        } catch (IOException e) {
            lastError = e.getMessage() == null ? e.toString() : e.getMessage();
            return -1;
        } catch (Throwable t) {
            // Nothing here may escape into native code: a Java exception unwinding
            // through JNI is a crash, and this app has to be able to say what went
            // wrong on the panel instead.
            lastError = t.toString();
            return -1;
        } finally {
            if (input != null) {
                try {
                    input.close();
                } catch (IOException ignored) {
                    // Closing a socket that already failed is not news.
                }
            }
            if (connection != null) {
                connection.disconnect();
            }
        }
    }
}
