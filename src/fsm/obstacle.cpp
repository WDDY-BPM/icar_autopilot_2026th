#include "fsm/obstacle.hpp"
#include "utils/tools.hpp"

FsmObstacle::FsmObstacle(std::shared_ptr<Params> par) : params(par) {}
FsmObstacle::~FsmObstacle() = default;

void FsmObstacle::run(Mat &img)
{
    (void)img;
    resultObs = PredictResult();
    if (params->pathOverride.active() &&
        params->pathOverride.source != PathSource::OBSTACLE)
        return;
    params->clearPathOverride(PathSource::OBSTACLE);
    if (!params->aiResultFresh)
        return;

    const auto &trackLeft = params->track->pointsEdgeLeft;
    const auto &trackRight = params->track->pointsEdgeRight;
    if (trackLeft.size() < ROWSIMAGE / 2 ||
        trackRight.size() < ROWSIMAGE / 2)
        return;

    vector<PredictResult> obstacles;
    for (const auto &result : params->results)
    {
        if ((result.type == LABEL_CONE || result.type == LABEL_PERSON) &&
            result.y + result.height > ROWSIMAGE * 0.4 &&
            result.height < 100 && result.width < 90 &&
            result.height > 20 && result.width > 20)
            obstacles.push_back(result);
    }
    if (obstacles.empty())
    {
        params->releasePlannerSafety(PathSource::OBSTACLE);
        return;
    }

    const auto nearest = std::max_element(obstacles.begin(), obstacles.end(),
        [](const PredictResult &left, const PredictResult &right) {
            return left.width * left.height < right.width * right.height;
        });
    resultObs = *nearest;

    size_t row = 0;
    int closestDistance = COLSIMAGE;
    for (size_t index = 0; index < trackLeft.size(); ++index)
    {
        const int distance = abs(resultObs.y - trackLeft[index].x);
        if (distance < closestDistance)
        {
            closestDistance = distance;
            row = index;
        }
    }
    row = std::min(row, trackRight.size() - 1);
    const int obstacleRight = resultObs.x + resultObs.width;
    const int leftColumn = trackLeft[row].y;
    const int rightColumn = trackRight[row].y;
    if (obstacleRight <= leftColumn || rightColumn <= resultObs.x)
        return;

    vector<PointX> left = trackLeft;
    vector<PointX> right = trackRight;
    row = std::min(row, std::min(left.size(), right.size()) - 1);
    const int distanceLeft = resultObs.x - left[row].y;
    const int distanceRight = right[row].y - obstacleRight;

    if (abs(distanceLeft) <= abs(distanceRight))
    {
        if (resultObs.type == LABEL_PERSON)
            curtailTracking(false, left, right);
        else
        {
            vector<PointX> points(4);
            points[0] = left[row / 2];
            points[1] = {resultObs.y + resultObs.height,
                         resultObs.x + resultObs.width * 2};
            points[2] = {resultObs.y + resultObs.height / 2,
                         resultObs.x + resultObs.width * 2};
            points[3] = resultObs.y > left.back().x
                ? left.back() : PointX(resultObs.y, obstacleRight);
            left.resize(row / 2);
            const auto repair = Bezier(0.01, points);
            left.insert(left.end(), repair.begin(), repair.end());
        }
    }
    else
    {
        if (resultObs.type == LABEL_PERSON)
            curtailTracking(true, left, right);
        else
        {
            vector<PointX> points(4);
            points[0] = right[row / 2];
            points[1] = {resultObs.y + resultObs.height,
                         resultObs.x - resultObs.width * 2};
            points[2] = {resultObs.y + resultObs.height / 2,
                         resultObs.x - resultObs.width * 2};
            points[3] = resultObs.y > right.back().x
                ? right.back() : PointX(resultObs.y, resultObs.x);
            right.resize(row / 2);
            const auto repair = Bezier(0.01, points);
            right.insert(right.end(), repair.begin(), repair.end());
        }
    }

    left.resize(static_cast<size_t>(left.size() * 0.7));
    right.resize(static_cast<size_t>(right.size() * 0.7));
    params->pathOverride.setEdges(
        PathSource::OBSTACLE, std::move(left), std::move(right),
        0.45f, std::min(params->config.velSlow, 0.15f), 1);
    params->ctrl.obstacleSlow = true;
}

void FsmObstacle::resetLap()
{
    resultObs = PredictResult();
    params->clearPathOverride(PathSource::OBSTACLE);
    params->releasePlannerSafety(PathSource::OBSTACLE);
}

void FsmObstacle::show(Mat &img)
{
    if (resultObs.x > 0 && resultObs.y > 0)
        cv::rectangle(img,
            cv::Rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height),
            cv::Scalar(0, 0, 255), 1);
}

void FsmObstacle::curtailTracking(bool leftSide, vector<PointX> &left,
                                  vector<PointX> &right)
{
    const size_t common = std::min(left.size(), right.size());
    left.resize(common);
    right.resize(common);
    if (leftSide)
    {
        for (size_t index = 0; index < common; ++index)
            right[index].y = (right[index].y + left[index].y) / 2;
    }
    else
    {
        for (size_t index = 0; index < common; ++index)
            left[index].y = (right[index].y + left[index].y) / 2;
    }
}
