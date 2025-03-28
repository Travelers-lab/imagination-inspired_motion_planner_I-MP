from sklearn.linear_model import LinearRegression
import numpy as np

def regression(X, y):
    model = LinearRegression()
    model.fit(X, y)
    w_ = model.coef_
    b_ = model.intercept_

    return [w_[0][0], w_[0][1], b_[0]]

def main():
    X = np.linspace(0, 10, num=30).reshape(-1, 1)
    Y = 3 + 2*X + np.random.randn(30, 1)
    print("X, {}Y:{}".format(X, Y))
    a, b = regression(X, Y)
    print("a:{}, b:{}".format(a, b))

if __name__ == "__main__":
    main()