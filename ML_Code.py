# استيراد المكتبات
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split, GridSearchCV, cross_val_score
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier, VotingClassifier
from sklearn.metrics import accuracy_score
from xgboost import XGBClassifier
from sklearn.impute import SimpleImputer
from sklearn.tree import DecisionTreeClassifier
import m2cgen as m2c

# تحميل البيانات
df = pd.read_csv("D://New folder/Medicaldataset.csv")

# تحديد المدخلات والمخرجات
X = df[['Heart rate']]  # استخدام عمود 'thalach' فقط
y = df['Result']     # الهدف

# معالجة القيم المفقودة
imputer = SimpleImputer(strategy="mean")
X_imputed = imputer.fit_transform(X)

# التحجيم
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X_imputed)

# تقسيم البيانات
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42)

# بناء النماذج
rf_model = RandomForestClassifier(random_state=42)
gb_model = GradientBoostingClassifier(random_state=42)
xgb_model = XGBClassifier(random_state=42)

# تجميع النماذج
voting_model = VotingClassifier(estimators=[('rf', rf_model), ('gb', gb_model), ('xgb', xgb_model)], voting='hard')

# تحسين المعلمات
param_grid = {
    'rf__n_estimators': [100, 200],
    'gb__learning_rate': [0.01, 0.1],
    'xgb__learning_rate': [0.01, 0.1]
}
grid_search = GridSearchCV(voting_model, param_grid, cv=5, n_jobs=-1)
grid_search.fit(X_train, y_train)

# نتائج النموذج
print(f"أفضل المعلمات: {grid_search.best_params_}")
y_pred = grid_search.best_estimator_.predict(X_test)
accuracy = accuracy_score(y_test, y_pred)
print(f"الدقة على بيانات الاختبار: {accuracy * 100:.2f}%")

# التحقق باستخدام Cross-Validation
cv_scores = cross_val_score(grid_search.best_estimator_, X_scaled, y, cv=5)
print(f"الدقة باستخدام Cross-Validation: {np.mean(cv_scores) * 100:.2f}%")

# تدريب شجرة قرار وتحويلها إلى كود C
clf = DecisionTreeClassifier(random_state=42, max_depth=3)  # تحديد عمق للتبسيط
clf.fit(X_train, y_train)

# حفظ معايير التحجيم
print("\nمعايير التحجيم:")
print("المتوسط (mean):", scaler.mean_[0])
print("الانحراف المعياري (scale):", scaler.scale_[0])

# تحويل النموذج إلى كود C
c_code = m2c.export_to_c(clf)
with open("D://New folder//model3.c", "w") as f:
    f.write(c_code)
