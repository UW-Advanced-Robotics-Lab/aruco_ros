import cv2
import cv2.aruco as aruco

# Define the Aruco dictionary and the marker ID
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)  # Original Aruco dictionary
marker_id = 582  # Set marker ID to 582
marker_size = 200  # Marker size in pixels

# Generate the marker
marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)

# Save the marker image
cv2.imwrite(f"aruco_marker_{marker_id}.png", marker_image)

# Display the marker
cv2.imshow(f"Aruco Marker ID {marker_id}", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
