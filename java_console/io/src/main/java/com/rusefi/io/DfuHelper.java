package com.rusefi.io;

import com.rusefi.FileLog;
import com.rusefi.RusEfiSignature;
import com.rusefi.SignatureHelper;
import com.rusefi.autoupdate.Autoupdate;
import com.rusefi.binaryprotocol.BinaryProtocol;
import com.rusefi.config.generated.Fields;

import javax.swing.*;
import java.io.IOException;

import static com.rusefi.Timeouts.SECOND;
import static com.rusefi.binaryprotocol.BinaryProtocol.sleep;

public class DfuHelper {
    public static void sendDfuRebootCommand(IoStream stream, StringBuilder messages) {
        byte[] command = BinaryProtocol.getTextCommandBytes(Fields.CMD_REBOOT_DFU);
        try {
            stream.sendPacket(command);
            stream.close();
            messages.append("Reboot command sent!\n");
        } catch (IOException e) {
            messages.append("Error " + e);
        }
    }

    public static boolean sendDfuRebootCommand(JComponent parent, String signature, IoStream stream, StringBuilder messages) {
        RusEfiSignature s = SignatureHelper.parse(signature);
        String bundleName = Autoupdate.readBundleFullName();
        if (bundleName != null && s != null) {
            if (!bundleName.equalsIgnoreCase(s.getBundle())) {
                String message = String.format("You have \"%s\" controller does not look right to program it with \"%s\"", s.getBundle(), bundleName);
                FileLog.MAIN.logLine(message);
                JOptionPane.showMessageDialog(parent, message);
                // in case of mismatched bundle type we are supposed do close connection
                // and properly handle the case of user hitting "Update Firmware" again
                // closing connection is a mess on Windows so it's simpler to just exit
                new Thread(() -> {
                    // let's have a delay and separate thread to address
                    // "wrong bundle" warning text sometimes not visible #3267
                    sleep(5 * SECOND);
                    System.exit(-5);
                }).start();
                return false;
            }
        }

        sendDfuRebootCommand(stream, messages);
        return true;
    }
}
