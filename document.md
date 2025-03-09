---
layout: default
title: Dataset
---

<div class="dataset-page">
    <div class="dataset-sidebar">
        <ul class="dataset-nav">
            <li><a href="#数据集" class="active">数据集</a></li>
            <li><a href="#数据处理">数据处理</a></li>
        </ul>
    </div>

    <div class="dataset-content">
        <div id="数据集">
            <h1>数据集</h1>
              内容
        </div>

        <div id="数据处理" style="display: none;">
            <h1>数据处理</h1>
        </div>

    </div>
</div>

<script>
document.querySelectorAll('.dataset-nav a').forEach(link => {
    link.addEventListener('click', function(e) {
        e.preventDefault();
        // 隐藏所有内容
        document.querySelectorAll('.dataset-content > div').forEach(div => {
            div.style.display = 'none';
        });
        // 显示选中的内容
        document.querySelector(this.getAttribute('href')).style.display = 'block';
        // 更新active状态
        document.querySelectorAll('.dataset-nav a').forEach(a => {
            a.classList.remove('active');
        });
        this.classList.add('active');
    });
});
</script>

<footer style="text-align: center; margin-top: 20px; padding: 10px; background-color: #f5f5f5;">
    <p>© Copyright Capital Normal University 2025</p>
</footer>


