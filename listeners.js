/**
 * File: listeners.js
 * Project: 3D Rigid Body Physics Engine - Collisions and Motion
 * Description: Initializes the event listeners for sliders and the refresh button.
 * Reponsible for retriving and updating values
 * Author: Alberto Chavez Garcia
 * Created: July 17, 2023
 * Last Modified: August 18, 2023
 */
 
document.addEventListener('DOMContentLoaded', () => {
    const sliders = document.querySelectorAll('input[type="range"]');
    let num_robots_create = 0;

    const url_params = new URLSearchParams(window.location.search);
    const url_num_robots = url_params.get('num_robots');
    if (url_num_robots !== null) {
        num_robots_create = parseInt(url_num_robots);
    }

    sliders.forEach(slider => {
        const slider_id = slider.id;
        const values_span = document.getElementById(`${slider_id}_value`);

        if (slider_id === 'piston_output') {
            values_span.textContent = (slider.value / 100).toFixed(2);
        } else if (slider_id === 'num_robots') {
            values_span.textContent = `${slider.value}`;
            num_robots_create = parseInt(slider.value);
        } else {
            values_span.textContent = slider.value;
        }

        slider.addEventListener('input', () => {
            if (slider_id === 'piston_output') {
                values_span.textContent = (slider.value / 100).toFixed(2);
            } else if (slider_id === 'num_robots') {
                values_span.textContent = `${slider.value}`;
                num_robots_create = parseInt(slider.value);

                const update_url = `${window.location.pathname}?num_robots=${num_robots_create}`;
                window.history.replaceState(null, null, update_url);
            } else {
                values_span.textContent = slider.value;
            }
        });
    });

    const refresh_button = document.getElementById('refresh_button');
    refresh_button.addEventListener('click', () => {
        location.reload();
    });
});