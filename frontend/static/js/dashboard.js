
<script>
function sendClick(event) { 
    const img = event.target; 
    const rect = img.getBoundingClientRect();

    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    fetch("/manual_click", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({
            x: x,
            y: y,
            width: rect.width,
            height: rect.height
        })
    });
}
</script>