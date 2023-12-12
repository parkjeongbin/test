#!/usr/bin/python3
import subprocess
from flask import Flask, render_template, jsonify

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('shop_list2.html')

@app.route('/run_script', methods=['GET'])
def run_script():
    try:
        # 1.py 스크립트 실행
        result = subprocess.run(['python3', '2.py'], capture_output=True, text=True)
        if result.returncode == 0:
            return jsonify(success=True, output=result.stdout)
        else:
            return jsonify(success=False, error=result.stderr)
    except Exception as e:
        return jsonify(success=False, error=str(e))

if __name__ == '__main__':
    app.run()

