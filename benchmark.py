import subprocess
import re
import csv
import os
import math
import json

# Configuration for Test 1: Lookahead Window Size Benchmark
MAX_STEPS_LIST = [2, 4, 8, 12, 16, 24, 32]
FIXED_VEHICLE_COUNT = 15

# Configuration for Test 2: Vehicle Density Benchmark
VEHICLE_COUNT_LIST = [5, 10, 15, 20, 25, 30, 35, 40]
FIXED_LOOKAHEAD = 8

# Shared parameters
TRIALS_PER_CONFIG = 15
SIM_STEPS = 50

def create_headless_source():
    print("Preparing headless source code...")
    if not os.path.exists("main.c"):
        raise FileNotFoundError("Could not find main.c in the current directory.")
        
    with open("main.c", "r") as f:
        content = f.read()
    
    # 1. Bypass animation delay
    content = content.replace("usleep(300000);", "/* usleep bypassed */")
    
    # 2. Bypass terminal screen clearing
    content = content.replace('printf("\\033[2J\\033[H");', "/* clear bypassed */")
    
    # 3. Bypass rendering of the grid
    content = content.replace('draw_scene(roads, roadNum, vehicles, vehiNum, side);', "/* draw bypassed */")
    
    # 4. Bypass terminal window resizing call
    content = content.replace("resize_terminal(canvaSide + vehiNum + 10, canvaSide + 40);", "/* resize bypassed */")
    
    # 5. Bypass the final getchar() hanging check
    content = content.replace("getchar();", "/* getchar bypassed */")
    
    with open("main_headless.c", "w") as f:
        f.write(content)

def compile_headless():
    print("Compiling headless binary...")
    subprocess.run(["gcc", "-O3", "main_headless.c", "-o", "main_headless"], check=True)

def run_lookahead_benchmark():
    print("\n=== Running Test 1: Lookahead Window Size Benchmark ===")
    raw_results = []
    
    for max_steps in MAX_STEPS_LIST:
        for trial in range(TRIALS_PER_CONFIG):
            print(f"[Test 1] Running lookahead={max_steps}, trial={trial+1}/{TRIALS_PER_CONFIG}...")
            cmd = ["./main_headless", "--random", str(FIXED_VEHICLE_COUNT), "--steps", str(SIM_STEPS), "--lookahead", str(max_steps), "--child"]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            output = proc.stdout
            
            plan_times = [float(x) for x in re.findall(r"plan (\d+\.\d+) ms", output)]
            avg_plan_time = sum(plan_times) / len(plan_times) if plan_times else 0.0
            
            final_section = output.split("───────────────────────────────────────────────────────────")[-1]
            goals_reached = len(re.findall(r"✔ GOAL", final_section))
            success_rate = goals_reached / FIXED_VEHICLE_COUNT
            
            raw_results.append({
                "maxSteps": max_steps,
                "trial": trial,
                "avg_plan_time_ms": avg_plan_time,
                "success_rate": success_rate,
                "goals_reached": goals_reached
            })
            
    csv_file = "benchmark_results.csv"
    with open(csv_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["maxSteps", "trial", "avg_plan_time_ms", "success_rate", "goals_reached"])
        writer.writeheader()
        writer.writerows(raw_results)
    print(f"Test 1 CSV results saved to {csv_file}")
    return raw_results

def run_vehicles_benchmark():
    print("\n=== Running Test 2: Vehicle Density Benchmark ===")
    raw_results = []
    
    for v_count in VEHICLE_COUNT_LIST:
        for trial in range(TRIALS_PER_CONFIG):
            print(f"[Test 2] Running vehicle_count={v_count}, trial={trial+1}/{TRIALS_PER_CONFIG}...")
            cmd = ["./main_headless", "--random", str(v_count), "--steps", str(SIM_STEPS), "--lookahead", str(FIXED_LOOKAHEAD), "--child"]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            output = proc.stdout
            
            plan_times = [float(x) for x in re.findall(r"plan (\d+\.\d+) ms", output)]
            avg_plan_time = sum(plan_times) / len(plan_times) if plan_times else 0.0
            
            final_section = output.split("───────────────────────────────────────────────────────────")[-1]
            goals_reached = len(re.findall(r"✔ GOAL", final_section))
            success_rate = goals_reached / v_count if v_count > 0 else 0.0
            
            raw_results.append({
                "vehicleCount": v_count,
                "trial": trial,
                "avg_plan_time_ms": avg_plan_time,
                "success_rate": success_rate,
                "goals_reached": goals_reached
            })
            
    csv_file = "benchmark_vehicles_results.csv"
    with open(csv_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["vehicleCount", "trial", "avg_plan_time_ms", "success_rate", "goals_reached"])
        writer.writeheader()
        writer.writerows(raw_results)
    print(f"Test 2 CSV results saved to {csv_file}")
    return raw_results

def compute_statistics_lookahead(raw_results):
    stats = {}
    for r in raw_results:
        ms = r["maxSteps"]
        if ms not in stats:
            stats[ms] = {"success_rates": [], "plan_times": []}
        stats[ms]["success_rates"].append(r["success_rate"])
        stats[ms]["plan_times"].append(r["avg_plan_time_ms"])
        
    summary = []
    for ms in MAX_STEPS_LIST:
        s_list = stats[ms]["success_rates"]
        p_list = stats[ms]["plan_times"]
        
        mean_s = sum(s_list) / len(s_list)
        var_s = sum((x - mean_s) ** 2 for x in s_list) / (len(s_list) - 1) if len(s_list) > 1 else 0.0
        ci_s = 1.96 * math.sqrt(var_s / len(s_list)) if len(s_list) > 0 else 0.0
        
        mean_p = sum(p_list) / len(p_list)
        var_p = sum((x - mean_p) ** 2 for x in p_list) / (len(p_list) - 1) if len(p_list) > 1 else 0.0
        ci_p = 1.96 * math.sqrt(var_p / len(p_list)) if len(p_list) > 0 else 0.0
        
        summary.append({
            "maxSteps": ms,
            "mean_success_rate": mean_s,
            "ci_success_rate": ci_s,
            "mean_plan_time_ms": mean_p,
            "ci_plan_time_ms": ci_p
        })
    return summary

def compute_statistics_vehicles(raw_results):
    stats = {}
    for r in raw_results:
        vc = r["vehicleCount"]
        if vc not in stats:
            stats[vc] = {"success_rates": [], "plan_times": []}
        stats[vc]["success_rates"].append(r["success_rate"])
        stats[vc]["plan_times"].append(r["avg_plan_time_ms"])
        
    summary = []
    for vc in VEHICLE_COUNT_LIST:
        s_list = stats[vc]["success_rates"]
        p_list = stats[vc]["plan_times"]
        
        mean_s = sum(s_list) / len(s_list)
        var_s = sum((x - mean_s) ** 2 for x in s_list) / (len(s_list) - 1) if len(s_list) > 1 else 0.0
        ci_s = 1.96 * math.sqrt(var_s / len(s_list)) if len(s_list) > 0 else 0.0
        
        mean_p = sum(p_list) / len(p_list)
        var_p = sum((x - mean_p) ** 2 for x in p_list) / (len(p_list) - 1) if len(p_list) > 1 else 0.0
        ci_p = 1.96 * math.sqrt(var_p / len(p_list)) if len(p_list) > 0 else 0.0
        
        summary.append({
            "vehicleCount": vc,
            "mean_success_rate": mean_s,
            "ci_success_rate": ci_s,
            "mean_plan_time_ms": mean_p,
            "ci_plan_time_ms": ci_p
        })
    return summary

def generate_html_dashboard(summary_lookahead, summary_vehicles):
    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>WHCA* Performance Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {{
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            background-color: #f8fafc;
            color: #0f172a;
            margin: 0;
            padding: 40px 20px;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
        }}
        header {{
            text-align: center;
            margin-bottom: 40px;
        }}
        h1 {{
            font-size: 2.5rem;
            color: #0284c7;
            margin-bottom: 10px;
            font-weight: 700;
        }}
        p.subtitle {{
            color: #475569;
            font-size: 1.1rem;
        }}
        .section-header {{
            margin-top: 50px;
            margin-bottom: 25px;
            padding-bottom: 10px;
            border-bottom: 3px solid #0284c7;
        }}
        .section-title {{
            font-size: 1.8rem;
            color: #0f172a;
            margin: 0 0 8px 0;
            font-weight: 700;
        }}
        .section-desc {{
            color: #64748b;
            font-size: 1rem;
            margin: 0;
        }}
        .grid {{
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 30px;
            margin-bottom: 30px;
        }}
        @media (max-width: 900px) {{
            .grid {{
                grid-template-columns: 1fr;
            }}
        }}
        .card {{
            background-color: #ffffff;
            border-radius: 12px;
            padding: 24px;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05), 0 2px 4px -2px rgba(0, 0, 0, 0.05);
            border: 1px solid #e2e8f0;
            margin-bottom: 30px;
        }}
        h2 {{
            font-size: 1.3rem;
            margin-top: 0;
            margin-bottom: 20px;
            color: #0f172a;
            border-bottom: 2px solid #f1f5f9;
            padding-bottom: 10px;
            font-weight: 600;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin-top: 10px;
        }}
        th, td {{
            padding: 12px 16px;
            text-align: left;
            border-bottom: 1px solid #e2e8f0;
        }}
        th {{
            background-color: #f1f5f9;
            color: #0369a1;
            font-weight: 600;
            font-size: 0.9rem;
            text-transform: uppercase;
            letter-spacing: 0.05em;
        }}
        tr:hover {{
            background-color: #f8fafc;
        }}
        td {{
            color: #334155;
            font-size: 0.95rem;
        }}
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>WHCA* Performance Dashboard</h1>
            <p class="subtitle">Comprehensive statistical evaluation of pathfinding efficiency across varying lookahead windows and vehicle densities.</p>
        </header>

        <!-- Test 1: Lookahead Benchmark -->
        <div class="section-header">
            <h2 class="section-title">Test 1: Lookahead Horizon Benchmark (maxSteps)</h2>
            <p class="section-desc">Evaluates algorithm performance under varying lookahead window sizes (maxSteps) with a fixed density of {FIXED_VEHICLE_COUNT} vehicles.</p>
        </div>

        <div class="grid">
            <div class="card">
                <h2>Goal Success Rate (%) vs. maxSteps</h2>
                <canvas id="successChartLookahead"></canvas>
            </div>
            <div class="card">
                <h2>Planning Time (ms/tick) vs. maxSteps</h2>
                <canvas id="timeChartLookahead"></canvas>
            </div>
        </div>

        <div class="card">
            <h2>Test 1 Statistical Summary Table</h2>
            <table>
                <thead>
                    <tr>
                        <th>maxSteps</th>
                        <th>Mean Success Rate</th>
                        <th>95% Confidence Interval (Success)</th>
                        <th>Mean Planning Time (ms/tick)</th>
                        <th>95% Confidence Interval (Time)</th>
                    </tr>
                </thead>
                <tbody>
                    {"".join(f'''
                    <tr>
                        <td><strong>{item["maxSteps"]}</strong></td>
                        <td>{item["mean_success_rate"] * 100:.1f}%</td>
                        <td>&plusmn; {item["ci_success_rate"] * 100:.1f}%</td>
                        <td>{item["mean_plan_time_ms"]:.3f} ms</td>
                        <td>&plusmn; {item["ci_plan_time_ms"]:.3f} ms</td>
                    </tr>
                    ''' for item in summary_lookahead)}
                </tbody>
            </table>
        </div>

        <!-- Test 2: Vehicle Density Benchmark -->
        <div class="section-header">
            <h2 class="section-title">Test 2: Vehicle Density Benchmark (Random Vehicle Count)</h2>
            <p class="section-desc">Evaluates algorithm scalability and path completion success when scaling the number of randomly generated vehicles (random parameter) at a fixed lookahead horizon of {FIXED_LOOKAHEAD} steps.</p>
        </div>

        <div class="grid">
            <div class="card">
                <h2>Goal Success Rate (%) vs. Vehicle Count</h2>
                <canvas id="successChartVehicles"></canvas>
            </div>
            <div class="card">
                <h2>Planning Time (ms/tick) vs. Vehicle Count</h2>
                <canvas id="timeChartVehicles"></canvas>
            </div>
        </div>

        <div class="card">
            <h2>Test 2 Statistical Summary Table</h2>
            <table>
                <thead>
                    <tr>
                        <th>Vehicle Count (random)</th>
                        <th>Mean Success Rate</th>
                        <th>95% Confidence Interval (Success)</th>
                        <th>Mean Planning Time (ms/tick)</th>
                        <th>95% Confidence Interval (Time)</th>
                    </tr>
                </thead>
                <tbody>
                    {"".join(f'''
                    <tr>
                        <td><strong>{item["vehicleCount"]}</strong></td>
                        <td>{item["mean_success_rate"] * 100:.1f}%</td>
                        <td>&plusmn; {item["ci_success_rate"] * 100:.1f}%</td>
                        <td>{item["mean_plan_time_ms"]:.3f} ms</td>
                        <td>&plusmn; {item["ci_plan_time_ms"]:.3f} ms</td>
                    </tr>
                    ''' for item in summary_vehicles)}
                </tbody>
            </table>
        </div>

    </div>

    <script>
        const summaryLookahead = {json.dumps(summary_lookahead)};
        const labelsLookahead = summaryLookahead.map(d => d.maxSteps);
        
        // Test 1: Success Chart
        new Chart(document.getElementById('successChartLookahead').getContext('2d'), {{
            type: 'line',
            data: {{
                labels: labelsLookahead,
                datasets: [{{
                    label: 'Mean Success Rate',
                    data: summaryLookahead.map(d => d.mean_success_rate),
                    borderColor: '#0284c7',
                    backgroundColor: 'rgba(2, 132, 199, 0.08)',
                    tension: 0.1,
                    fill: true,
                    borderWidth: 3,
                    pointRadius: 6,
                    pointBackgroundColor: '#0284c7'
                }}]
            }},
            options: {{
                responsive: true,
                scales: {{
                    y: {{
                        min: 0, max: 1.0,
                        ticks: {{ callback: value => (value * 100) + '%', color: '#475569' }},
                        grid: {{ color: '#e2e8f0' }}
                    }},
                    x: {{
                        title: {{ display: true, text: 'maxSteps', color: '#475569' }},
                        ticks: {{ color: '#475569' }},
                        grid: {{ color: '#e2e8f0' }}
                    }}
                }},
                plugins: {{ legend: {{ labels: {{ color: '#0f172a' }} }} }}
            }}
        }});

        // Test 1: Planning Time Chart
        new Chart(document.getElementById('timeChartLookahead').getContext('2d'), {{
            type: 'line',
            data: {{
                labels: labelsLookahead,
                datasets: [{{
                    label: 'Mean Planning Time (ms)',
                    data: summaryLookahead.map(d => d.mean_plan_time_ms),
                    borderColor: '#e11d48',
                    backgroundColor: 'rgba(225, 29, 72, 0.08)',
                    tension: 0.1,
                    fill: true,
                    borderWidth: 3,
                    pointRadius: 6,
                    pointBackgroundColor: '#e11d48'
                }}]
            }},
            options: {{
                responsive: true,
                scales: {{
                    y: {{
                        title: {{ display: true, text: 'ms per tick', color: '#475569' }},
                        ticks: {{ color: '#475569' }},
                        grid: {{ color: '#e2e8f0' }}
                    }},
                    x: {{
                        title: {{ display: true, text: 'maxSteps', color: '#475569' }},
                        ticks: {{ color: '#475569' }},
                        grid: {{ color: '#e2e8f0' }}
                    }}
                }},
                plugins: {{ legend: {{ labels: {{ color: '#0f172a' }} }} }}
            }}
        }});

        const summaryVehicles = {json.dumps(summary_vehicles)};
        const labelsVehicles = summaryVehicles.map(d => d.vehicleCount);
        
        // Test 2: Success Chart
        new Chart(document.getElementById('successChartVehicles').getContext('2d'), {{
            type: 'line',
            data: {{
                labels: labelsVehicles,
                datasets: [{{
                    label: 'Mean Success Rate',
                    data: summaryVehicles.map(d => d.mean_success_rate),
                    borderColor: '#059669',
                    backgroundColor: 'rgba(5, 150, 105, 0.08)',
                    tension: 0.1,
                    fill: true,
                    borderWidth: 3,
                    pointRadius: 6,
                    pointBackgroundColor: '#059669'
                }}]
            }},
            options: {{
                responsive: true,
                scales: {{
                    y: {{
                        min: 0, max: 1.0,
                        ticks: {{ callback: value => (value * 100) + '%', color: '#475569' }},
                        grid: {{ color: '#e2e8f0' }}
                    }},
                    x: {{
                        title: {{ display: true, text: 'Random Vehicle Count', color: '#475569' }},
                        ticks: {{ color: '#475569' }},
                        grid: {{ color: '#e2e8f0' }}
                    }}
                }},
                plugins: {{ legend: {{ labels: {{ color: '#0f172a' }} }} }}
            }}
        }});

        // Test 2: Planning Time Chart
        new Chart(document.getElementById('timeChartVehicles').getContext('2d'), {{
            type: 'line',
            data: {{
                labels: labelsVehicles,
                datasets: [{{
                    label: 'Mean Planning Time (ms)',
                    data: summaryVehicles.map(d => d.mean_plan_time_ms),
                    borderColor: '#d97706',
                    backgroundColor: 'rgba(217, 119, 6, 0.08)',
                    tension: 0.1,
                    fill: true,
                    borderWidth: 3,
                    pointRadius: 6,
                    pointBackgroundColor: '#d97706'
                }}]
            }},
            options: {{
                responsive: true,
                scales: {{
                    y: {{
                        title: {{ display: true, text: 'ms per tick', color: '#475569' }},
                        ticks: {{ color: '#475569' }},
                        grid: {{ color: '#e2e8f0' }}
                    }},
                    x: {{
                        title: {{ display: true, text: 'Random Vehicle Count', color: '#475569' }},
                        ticks: {{ color: '#475569' }},
                        grid: {{ color: '#e2e8f0' }}
                    }}
                }},
                plugins: {{ legend: {{ labels: {{ color: '#0f172a' }} }} }}
            }}
        }});
    </script>
</body>
</html>
"""

    html_file = "performance_dashboard.html"
    with open(html_file, "w") as f:
        f.write(html_content)
    print(f"Interactive HTML dashboard saved to {html_file}")

if __name__ == "__main__":
    try:
        create_headless_source()
        compile_headless()
        
        # Test 1: Lookahead Horizon Benchmark
        raw_lookahead = run_lookahead_benchmark()
        summary_lookahead = compute_statistics_lookahead(raw_lookahead)
        
        # Test 2: Vehicle Density Benchmark
        raw_vehicles = run_vehicles_benchmark()
        summary_vehicles = compute_statistics_vehicles(raw_vehicles)
        
        generate_html_dashboard(summary_lookahead, summary_vehicles)
    finally:
        # Cleanup temporary files
        if os.path.exists("main_headless.c"):
            os.remove("main_headless.c")
        if os.path.exists("main_headless"):
            os.remove("main_headless")
        print("Cleanup of temporary files completed.")
