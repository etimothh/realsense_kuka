import java.io.*;
import java.net.*;
import java.awt.image.*;
import javax.imageio.ImageIO;

public class ImageReceiver {
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;

    public static void main(String[] args) {
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
            int dataSize;
            byte[] dataBuffer = new byte[1024];
            InputStream inputStream = imageSocket.getInputStream();
            DataInputStream dataInputStream = new DataInputStream(inputStream);

            // Read the size of the data
            dataSize = dataInputStream.readInt();
            System.out.println("Received data size: " + dataSize);

            // Read the data itself
            ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
            while (dataSize > 0) {
                int bytesRead = inputStream.read(dataBuffer, 0, Math.min(dataSize, dataBuffer.length));
                byteArrayOutputStream.write(dataBuffer, 0, bytesRead);
                dataSize -= bytesRead;
            }

            // Convert image data to BufferedImage
            BufferedImage image = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_RGB);
            byte[] imageBytes = byteArrayOutputStream.toByteArray();
            for (int y = 0; y < HEIGHT; y++) {
                for (int x = 0; x < WIDTH; x++) {
                    int index = (y * WIDTH + x) * 3;
                    int r = imageBytes[index] & 0xFF;
                    int g = imageBytes[index + 1] & 0xFF;
                    int b = imageBytes[index + 2] & 0xFF;
                    int rgb = (r << 16) | (g << 8) | b;
                    image.setRGB(x, y, rgb);
                }
            }

            // Save the image data to a file
            String filePath = "received_image.jpg";
            ImageIO.write(image, "jpg", new File(filePath));
            System.out.println("Image data saved to " + filePath);

            // Close image socket
            imageSocket.close();
            System.out.println("Image connection closed");

        } catch (IOException e) {
            System.err.println("Error: " + e.getMessage());
        }
    }
}
