import numpy as np


c_1 = 1e-4
c_2 = 0.9

def cost(x, y, w):
    power = -np.multiply(y, x.dot(w))
    p1 = power[power <= 0]
    p2 = -power[-power < 0]
    return np.sum(np.log(1 + np.exp(p1))) + np.sum(np.log(1 + np.exp(p2))-p2)

def dcost(x, y, w):
    return x.T.dot(np.multiply(-y, 1/(1 + np.exp(-np.multiply(y, x.dot(w))))))

def direction(d):
    return -d

def sufficientDecrease(x, y, w, step):
    d = dcost(x, y, w)
    p = direction(d)
    return cost(x, y, w+step*p) <= cost(x, y, w) + c_1*step*p.T.dot(d)

def curvature(x, y, w, step):
    d = dcost(x, y, w)
    p = direction(d)
    print("p:{}".format(-p.T.dot(dcost(x, y, w + step*p))))
    print("c:{}".format(-c_2*p.T.dot(d)))
    return -p.T.dot(dcost(x, y, w + step*p)) <= -c_2*p.T.dot(d)

def select(step_low, step_high):
    return (step_low + step_high)/2

def lineSearch(x, y, w, step_init, step_max):
    step_i = step_init
    step_low = step_init
    step_high = step_max
    i = 1
    d = dcost(x, y, w)
    p = direction(d)
    while(True):
        print("sufficient:{}".format(sufficientDecrease(x, y, w, step_i)))
        print("cost:{}".format(cost(x, y, w+step_i*p) >= cost(x, y, w+step_low*p) and i > 1))
        if (not sufficientDecrease(x, y, w, step_i) or (cost(x, y, w+step_i*p) >= cost(x, y, w+step_low*p) and i>1)):
            step_high = step_i
            #print("step:{}".format(step_high))
        else:
            #print("curvature:{}".format(curvature(x, y, w, step_i)))
            if (curvature(x, y, w, step_i)):
                return step_i
            step_low = step_i
        step_i = select(step_low, step_high)
        i += 1

def logisticRefressionGd(x, y, max_iter = 1000, tol=1e-4, step_init=0, step_max=50):
    w = np.ones(x.shape[1])
    for it in range(max_iter):
        d = dcost(x, y, w)
        print("d:{}".format(d))
        if np.linalg.norm(x=d, ord=1) <= tol:
            break
        step = lineSearch(x, y, w, step_init, step_max)
        #print("step:{}".format(step))
        w = w + step*direction(d)
    return w

def main():
    y = np.array([1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    x = np.array([[1,1], [2.1,1], [2.9,1], [3.5,1], [4.7,1], [5.5,1], [6.9,1], [8.1,1], [9.6,1], [10,1], [11,1], [11.5,1], [12.3,1], [13.7,1]])
    w = logisticRefressionGd(x, y)
    print(w)

if __name__ == "__main__":
    main()

