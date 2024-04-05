package application;

import java.util.ArrayList;


import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.SortieIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.geometricModel.Frame;
import java.net.*;
import java.io.*;
import java.awt.image.*;
import javax.imageio.ImageIO;

/* Ce code contient deux fonctions; capt_img(), demande au pc vlad connecter à la caméra de prendre une image et de lui envoyer le bitmap
 * send_img(byte[] imageData) envoie le bitmap donné en paramètre au programme sous vscode sur ce pc
 * in order to work properly, the programms on vscode and the vlad computer must be launched
 * 
 * */
public class receiver_sender extends RoboticsAPIApplication{
	private static final int WIDTH = 640;
	private static final int HEIGHT = 480;
	public void initialize() {
		// initialize your application here
	}

	@Override
	public void run() {
		// your application execution starts here
		byte[] img_byte=capt_img();         // save the captured image in img_byte
		send_img(img_byte);					// send the images to the vscode programm
	
	}
	
	public static byte[] capt_img() {
	    // TCP/IP connection settings
        String host = "172.31.1.149";
        int triggerPort = 12345;
        int imagePort = 12345;

        try {
            // Connect to the image sender
            System.out.println("Trying to connect to image sender at " + host + ":" + imagePort);
            Socket imageSocket = new Socket(host, imagePort);
            System.out.println("Connected to image sender at " + host + ":" + imagePort);

            // Receive image data
            System.out.println("Receiving image data...");
            byte[] dataBuffer = new byte[1024];
            InputStream inputStream = imageSocket.getInputStream();
            DataInputStream dataInputStream = new DataInputStream(inputStream);
            
            // Read the size of the data
            int dataSize = dataInputStream.readInt();
            System.out.println("Received data size: " + dataSize);

            // Read the data itself
            ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
            while (dataSize > 0) {
                int bytesRead = inputStream.read(dataBuffer, 0, Math.min(dataSize, dataBuffer.length));
                byteArrayOutputStream.write(dataBuffer, 0, bytesRead);
                dataSize -= bytesRead;
            }
            byte[] imageBytes = byteArrayOutputStream.toByteArray();
            // Convert image data to BufferedImage / previous coding
            /*
            BufferedImage image = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_RGB);
            for (int y = 0; y < HEIGHT; y++) {
                for (int x = 0; x < WIDTH; x++) {
                    int index = (y * WIDTH + x) * 3;
                    int b = imageBytes[index] & 0xFF;
                    int g = imageBytes[index + 1] & 0xFF;
                    int r = imageBytes[index + 2] & 0xFF;
                    int rgb = (r << 16) | (g << 8) | b;
                    image.setRGB(x, y, rgb);
                }
            }

            // Save the image data to a file
            String filePath = "received_image.jpg";
            ImageIO.write(image, "jpg", new File(filePath));
            System.out.println("Image data saved to " + filePath);
			*/
            // Close image socket
            imageSocket.close();
            System.out.println("Image connection closed");
            return(imageBytes);

        } catch (IOException e) {
            System.err.println("Error: " + e.getMessage());
		 }
		return null;
        
	}
	public static void send_img(byte[] imageData) {
	    // TCP/IP connection settings
        String host = "172.31.1.140";
        int triggerPort = 12345;
        int imagePort = 12345;
        int img_size=imageData.length;

        try {
            /*   old coding part to convert img path to img bitmap // not working properly
        	File file=new File(img_path);
        	FileInputStream fis = new FileInputStream(file);
        	ByteArrayOutputStream bos = new ByteArrayOutputStream();
        	byte[] buffer=new byte[img_size];
        	int bytesRead;
        	bytesRead=fis.read(buffer);
        	while (bytesRead!=-1){
        		bos.write(buffer,0,bytesRead);
        		bytesRead=fis.read(buffer);
        		System.out.println("biteareas   " + bytesRead);
        	}
        	
        	fis.close();
        	bos.close();
        	byte[] imageData= bos.toByteArray();
        	*/
        	// Connect to the image sender
            System.out.println("Trying to connect to image receiver at " + host + ":" + imagePort);
            Socket imageSocket = new Socket(host, imagePort);
            System.out.println("Connected to image receiver at " + host + ":" + imagePort);

            // Receive image data
            System.out.println("Sending image data...");
            byte[] dataBuffer = new byte[img_size];
            DataOutputStream outputStream =  new DataOutputStream(imageSocket.getOutputStream());
            
            // Send the size of the data
            outputStream.writeInt(imageData.length);
            System.out.println("Sent data size: " + imageData.length);

            // Send the data itself
            outputStream.write(imageData);
            if (imageData.length > 0) {
            	System.out.println(" image data sent");
            }
            // Close image socket
            imageSocket.close();
            System.out.println("Image connection closed");

        } catch (IOException e) {
            System.err.println("Error: " + e.getMessage());
		 }
	}	
}



