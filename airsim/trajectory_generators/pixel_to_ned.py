import cv2

def px_to_ned(x,y):
    north = 711./400*x -788429./400
    east = 353./199*y - 288447./199
    # print('x_ned = %d, y_ned = %d'%(x_ned, y_ned))
    return north, east

def greyscale_to_altitude(val):
    alt = -1207./600*val + 90653./600
    print(val)
    print('altitude = %f'%(alt))

def on_mouse(event,x,y,flags,param):
    global mouseX,mouseY,img
    if event == cv2.EVENT_LBUTTONDOWN:
        print('x = %d, y = %d'%(x, y))
        px_to_ned(x,y)
        greyscale_to_altitude(img[y,x,0])


# Used to get pixel values of specific ned locations

if __name__ == "__main__":
    cv2.namedWindow('image')
    cv2.setMouseCallback('image',on_mouse)

    while(1):
        img = cv2.imread("./path_gen/altitude_map.png")
        cv2.imshow('image',img)
        k = cv2.waitKey()
        if k == ord('x'):
            break

