"""Detection only. Drawing is performed after all reads of the original image."""
import math
import settings as cfg


def choose_unique(candidates, anchor, radius):
    eligible = [c for c in candidates if math.hypot(c[0]-anchor[0], c[1]-anchor[1]) <= radius]
    # Multiple compatible objects are ambiguous. Never silently switch tracks.
    return eligible[0] if len(eligible) == 1 else None


def cluster_rings(circles):
    groups = []
    for x, y, radius in circles:
        for group in groups:
            gx = sum(c[0] for c in group)/len(group)
            gy = sum(c[1] for c in group)/len(group)
            if math.hypot(x-gx, y-gy) < cfg.RING_CENTER_TOLERANCE:
                group.append((x,y,radius))
                break
        else:
            groups.append([(x,y,radius)])
    result = []
    for group in groups:
        radii = []
        for r in sorted(c[2] for c in group):
            if not radii or r-radii[-1] > 3:
                radii.append(r)
        if len(radii) < cfg.RING_MIN_DISTINCT_RADII:
            continue
        x,y = (sum(c[k] for c in group)/len(group) for k in (0,1))
        result.append((int(round(x)), int(round(y)), 80, int(max(radii)*2), int(max(radii)*2)))
    return sorted(result, key=lambda c: c[0])


def material(img, request):
    x,y,w,h,u,v,_,_ = request['values']
    bounds = cfg.SIZE_BOUNDS.get(request['mode'])
    if bounds is None:
        return None
    candidates = []
    for blob in img.find_blobs([cfg.THRESHOLDS[request['color']]], roi=(x,y,w,h),
                               area_threshold=50, pixels_threshold=50):
        area = blob.w()*blob.h()
        if not cfg.MATERIAL_AREA[0] <= area <= cfg.MATERIAL_AREA[1]:
            continue
        if not bounds[0] <= blob.w() <= bounds[1] or not bounds[2] <= blob.h() <= bounds[3]:
            continue
        # Clipped blobs cannot provide a reliable center/shape.
        if blob.x() <= x or blob.y() <= y or blob.x()+blob.w() >= x+w or blob.y()+blob.h() >= y+h:
            continue
        fill = blob.pixels()/area
        if fill < cfg.MIN_FILL:
            continue
        candidates.append((blob.cx(), blob.cy(), min(100,int(fill*100)),blob.w(),blob.h()))
    radius = cfg.VERIFY_TOLERANCE if request['mode'] in (3,4) else cfg.ALIGN_SEARCH_RADIUS
    return choose_unique(candidates, (u,v), radius)


def ring(img, request):
    import cv2
    from maix import image
    x,y,w,h,u,v,_,_ = request['values']
    gray = image.image2cv(img.to_format(image.Format.FMT_GRAYSCALE), False, False)
    roi = gray[y:y+h,x:x+w]
    if min(roi.shape[:2]) < 23:
        return None
    binary = cv2.adaptiveThreshold(cv2.GaussianBlur(roi,(3,3),0),255,
                                  cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,21,10)
    contours,hierarchy = cv2.findContours(binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    circles = []
    if hierarchy is None:
        return None
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour); perimeter = cv2.arcLength(contour,True)
        if area <= 0 or perimeter <= 0 or 4*math.pi*area/perimeter**2 < 0.6:
            continue
        # Require a parent or child: isolated filled discs are not ring targets.
        if hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0:
            continue
        (cx,cy),r = cv2.minEnclosingCircle(contour)
        if not cfg.RING_MIN_RADIUS <= r <= cfg.RING_MAX_RADIUS:
            continue
        bx,by,bw,bh = cv2.boundingRect(contour)
        if bx <= 0 or by <= 0 or bx+bw >= w or by+bh >= h:
            continue
        m=cv2.moments(contour)
        circles.append((x+m['m10']/m['m00'],y+m['m01']/m['m00'],r))
    # Identity comes from calibrated station pose + restricted ROI, NOT sorting.
    selected=choose_unique(cluster_rings(circles),(u,v),cfg.ALIGN_SEARCH_RADIUS)
    if selected is None:
        return None
    # This version only places onto EMPTY rings, not stacking. Reject a dense
    # colored object covering the selected center. Thin black ring lines/digits
    # must be separated from material blobs during site calibration.
    for threshold in cfg.THRESHOLDS.values():
        for b in img.find_blobs([threshold],roi=(x,y,w,h),area_threshold=50,pixels_threshold=50):
            area=b.w()*b.h()
            if (area>=cfg.MATERIAL_AREA[0] and b.pixels()/max(area,1)>=cfg.MIN_FILL and
                b.x()<=selected[0]<=b.x()+b.w() and b.y()<=selected[1]<=b.y()+b.h()):
                return None
    return selected
